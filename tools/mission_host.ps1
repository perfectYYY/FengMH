param(
    [string]$Port = 'COM27',
    [int]$Baud = 115200,
    [string]$ConfigPath = "$PSScriptRoot\mission_config.json",
    [double]$DriveSpeed = 0.5,
    [double]$ArmTimeoutSeconds = 60.0,
    [double]$PositionToleranceM = 0.04,
    [switch]$ResumeAfterArmBox1Placed,
    [switch]$ValidateOnly,
    [switch]$Execute
)

$ErrorActionPreference = 'Stop'

# FengMH USB CDC protocol
$HEAD1 = [byte]0x55
$HEAD2 = [byte]0xAA
$FUNC_CHASSIS = [byte]0x10
$FUNC_ARM_TARGET = [byte]0x11
$FUNC_ARM_PUMP = [byte]0x14
$FUNC_MODE = [byte]0x15
$FUNC_ARM_AUX = [byte]0x17
$FUNC_ARM_FEEDBACK = [byte]0x86
$FUNC_MODE_FEEDBACK = [byte]0x8B

$MODE_NAV = [byte]1
$MODE_ARM = [byte]2
$MODE_PLACE = [byte]5

$TARGET_GRASP = [byte]0
$TARGET_PLACE = [byte]1

$AUX_PC8 = [byte]0
$AUX_PA8 = [byte]2

$ARM_IDLE = [byte]0
$ARM_MOVING = [byte]1
$ARM_REACHED = [byte]2
$ARM_ERROR = [byte]3

$script:Serial = $null
$script:Rx = New-Object 'System.Collections.Generic.List[byte]'
$script:FeedbackMode = -1
$script:FeedbackSafetyStop = -1
$script:ModeFeedbackAt = [datetime]::MinValue
$script:ArmState = -1
$script:ArmX = [double]::NaN
$script:ArmY = [double]::NaN
$script:ArmZ = [double]::NaN
$script:ArmFeedbackAt = [datetime]::MinValue
$script:MissionFailed = $false
$script:Mission = $null

function Get-ConfigPoint([object]$Point, [string]$Name) {
    if ($null -eq $Point -or $Point.Count -ne 3) {
        throw "Config field '$Name' must contain exactly 3 numbers [x, y, z]."
    }
    $result = [double[]]@($Point[0], $Point[1], $Point[2])
    $radius = [Math]::Sqrt(($result[0] * $result[0]) +
                           ($result[1] * $result[1]) +
                           ($result[2] * $result[2]))
    if ($radius -lt 0.045 -or $radius -gt 0.655) {
        throw "Config point '$Name' is outside the MCU reachable range (radius=$radius m)."
    }
    return $result
}

function Load-MissionConfig {
    if (-not (Test-Path -LiteralPath $ConfigPath)) {
        throw "Mission config not found: $ConfigPath"
    }
    $raw = Get-Content -LiteralPath $ConfigPath -Raw | ConvertFrom-Json
    foreach ($field in @('FirstNavDistanceM', 'SecondNavDistanceM')) {
        $value = [double]$raw.$field
        if ($value -le 0.0 -or $value -gt 20.0) {
            throw "Config field '$field' must be within (0, 20] meters."
        }
    }
    $script:Mission = [pscustomobject]@{
        FirstNavDistanceM = [double]$raw.FirstNavDistanceM
        SecondNavDistanceM = [double]$raw.SecondNavDistanceM
        ArmGrasp1 = Get-ConfigPoint $raw.ArmGrasp1 'ArmGrasp1'
        ArmPlace1 = Get-ConfigPoint $raw.ArmPlace1 'ArmPlace1'
        ArmGrasp2 = Get-ConfigPoint $raw.ArmGrasp2 'ArmGrasp2'
        ArmPlace2 = Get-ConfigPoint $raw.ArmPlace2 'ArmPlace2'
        PlaceGrasp1 = Get-ConfigPoint $raw.PlaceGrasp1 'PlaceGrasp1'
        PlacePlace1 = Get-ConfigPoint $raw.PlacePlace1 'PlacePlace1'
        PlaceGrasp2 = Get-ConfigPoint $raw.PlaceGrasp2 'PlaceGrasp2'
        PlacePlace2 = Get-ConfigPoint $raw.PlacePlace2 'PlacePlace2'
    }
    Write-Step ("Loaded mission config: nav={0:F3} m / {1:F3} m, config={2}" -f
                $script:Mission.FirstNavDistanceM,
                $script:Mission.SecondNavDistanceM,
                (Resolve-Path -LiteralPath $ConfigPath))
}

function Write-Step([string]$Message) {
    Write-Output ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss.fff'), $Message)
}

function New-Frame([byte]$Func, [byte[]]$Payload) {
    $length = [byte]$Payload.Length
    $sum = [int]$HEAD1 + [int]$HEAD2 + [int]$Func + [int]$length
    foreach ($b in $Payload) { $sum += [int]$b }
    return [byte[]](@($HEAD1, $HEAD2, $Func, $length) +
                    $Payload +
                    [byte]($sum -band 0xFF))
}

function Write-Frame([byte[]]$Frame) {
    try {
        $script:Serial.Write($Frame, 0, $Frame.Length)
    }
    catch {
        Write-Step ("Serial write interrupted: {0}; reconnecting..." -f $_.Exception.Message)
        Reconnect-Serial
        $script:Serial.Write($Frame, 0, $Frame.Length)
    }
}

function New-OpenedSerial {
    $portObject = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,one
    $portObject.ReadTimeout = 50
    $portObject.WriteTimeout = 500
    $portObject.Open()
    return $portObject
}

function Reconnect-Serial {
    if ($null -ne $script:Serial) {
        try { if ($script:Serial.IsOpen) { $script:Serial.Close() } } catch {}
        try { $script:Serial.Dispose() } catch {}
    }

    $deadline = (Get-Date).AddSeconds(10)
    $lastError = $null
    while ((Get-Date) -lt $deadline) {
        try {
            $script:Serial = New-OpenedSerial
            $script:Rx.Clear()
            Write-Step "Reconnected to $Port."
            return
        }
        catch {
            $lastError = $_.Exception.Message
            Start-Sleep -Milliseconds 500
        }
    }
    throw "Unable to reconnect to $Port within 10 s: $lastError"
}

function Send-Mode([byte]$Mode) {
    Write-Frame (New-Frame $FUNC_MODE ([byte[]]@($Mode)))
}

function Send-Chassis([double]$Vx, [double]$Vy = 0.0, [double]$Wz = 0.0) {
    $payload = [byte[]](
        [BitConverter]::GetBytes([single]$Vx) +
        [BitConverter]::GetBytes([single]$Vy) +
        [BitConverter]::GetBytes([single]$Wz)
    )
    Write-Frame (New-Frame $FUNC_CHASSIS $payload)
}

function Send-ArmTarget([byte]$Type, [double]$X, [double]$Y, [double]$Z) {
    $payload = [byte[]](
        @($Type) +
        [BitConverter]::GetBytes([single]$X) +
        [BitConverter]::GetBytes([single]$Y) +
        [BitConverter]::GetBytes([single]$Z)
    )
    Write-Frame (New-Frame $FUNC_ARM_TARGET $payload)
}

function Send-MainPump([bool]$On) {
    $value = if ($On) { [byte]1 } else { [byte]0 }
    for ($i = 0; $i -lt 3; $i++) {
        Write-Frame (New-Frame $FUNC_ARM_PUMP ([byte[]]@($value)))
        Start-Sleep -Milliseconds 20
    }
    Write-Step ("PD11 main pump -> {0}" -f $(if ($On) { 'ON' } else { 'OFF' }))
}

function Send-AuxPump([byte]$Channel, [bool]$On, [string]$Name) {
    $value = if ($On) { [byte]1 } else { [byte]0 }
    for ($i = 0; $i -lt 3; $i++) {
        Write-Frame (New-Frame $FUNC_ARM_AUX ([byte[]]@($Channel, $value)))
        Start-Sleep -Milliseconds 20
    }
    Write-Step ("{0} auxiliary pump -> {1}" -f $Name, $(if ($On) { 'ON' } else { 'OFF' }))
}

function Read-Frames([int]$WaitMilliseconds = 20) {
    $frames = @()
    $deadline = (Get-Date).AddMilliseconds($WaitMilliseconds)

    do {
        try {
            $available = $script:Serial.BytesToRead
            if ($available -gt 0) {
                $buffer = New-Object byte[] $available
                $read = $script:Serial.Read($buffer, 0, $buffer.Length)
                for ($i = 0; $i -lt $read; $i++) {
                    $script:Rx.Add($buffer[$i])
                }
            }
        } catch {
            Write-Step ("Serial read interrupted: {0}; reconnecting..." -f $_.Exception.Message)
            Reconnect-Serial
        }

        while ($script:Rx.Count -ge 5) {
            if ($script:Rx[0] -ne $HEAD1 -or $script:Rx[1] -ne $HEAD2) {
                $script:Rx.RemoveAt(0)
                continue
            }

            $length = [int]$script:Rx[3]
            $total = $length + 5
            if ($script:Rx.Count -lt $total) { break }

            $sum = 0
            for ($i = 0; $i -lt (4 + $length); $i++) {
                $sum += [int]$script:Rx[$i]
            }
            $expected = [byte]($sum -band 0xFF)
            $received = [byte]$script:Rx[4 + $length]

            if ($received -eq $expected) {
                $payload = New-Object byte[] $length
                for ($i = 0; $i -lt $length; $i++) {
                    $payload[$i] = $script:Rx[4 + $i]
                }
                $frames += [pscustomobject]@{
                    Func = [byte]$script:Rx[2]
                    Payload = $payload
                }
                $script:Rx.RemoveRange(0, $total)
            } else {
                $script:Rx.RemoveAt(0)
            }
        }

        if ((Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 2 }
    } while ((Get-Date) -lt $deadline)

    return $frames
}

function Update-Feedback {
    foreach ($frame in @(Read-Frames 20)) {
        if ($frame.Func -eq $FUNC_ARM_FEEDBACK -and $frame.Payload.Length -eq 17) {
            $script:ArmState = [int]$frame.Payload[0]
            $script:ArmX = [double][BitConverter]::ToSingle($frame.Payload, 1)
            $script:ArmY = [double][BitConverter]::ToSingle($frame.Payload, 5)
            $script:ArmZ = [double][BitConverter]::ToSingle($frame.Payload, 9)
            $script:ArmFeedbackAt = Get-Date
        } elseif ($frame.Func -eq $FUNC_MODE_FEEDBACK -and $frame.Payload.Length -eq 2) {
            $script:FeedbackMode = [int]$frame.Payload[0]
            $script:FeedbackSafetyStop = [int]$frame.Payload[1]
            $script:ModeFeedbackAt = Get-Date
        }
    }

    if ($script:FeedbackSafetyStop -eq 1) {
        throw 'MCU reports safety_stop_active=1; mission aborted.'
    }
}

function Set-ModeConfirmed([byte]$Mode, [string]$Name) {
    Write-Step "Request mode $Name ($Mode)"
    $started = Get-Date
    $nextSend = [datetime]::MinValue
    while (((Get-Date) - $started).TotalSeconds -lt 4.0) {
        if ((Get-Date) -ge $nextSend) {
            Send-Mode $Mode
            $nextSend = (Get-Date).AddMilliseconds(100)
        }
        Update-Feedback
        if ($script:FeedbackMode -eq $Mode -and
            $script:ModeFeedbackAt -ge $started -and
            $script:FeedbackSafetyStop -eq 0) {
            Write-Step "Mode confirmed: $Name"
            return
        }
    }
    throw "Mode $Name was not confirmed by 0x8B feedback."
}

function Stop-Chassis([byte]$Mode, [int]$Repeat = 10) {
    for ($i = 0; $i -lt $Repeat; $i++) {
        Send-Mode $Mode
        Send-Chassis 0.0
        Update-Feedback
        Start-Sleep -Milliseconds 20
    }
}

function Drive-Distance([double]$DistanceM) {
    if ($DriveSpeed -le 0.0) { throw 'DriveSpeed must be positive.' }
    $duration = $DistanceM / $DriveSpeed
    Write-Step ("NAV drive: vx={0:F3} m/s, distance={1:F3} m, timed duration={2:F3} s" -f
                $DriveSpeed, $DistanceM, $duration)

    $clock = [Diagnostics.Stopwatch]::StartNew()
    $nextModeMs = 0.0
    while ($clock.Elapsed.TotalSeconds -lt $duration) {
        if ($clock.Elapsed.TotalMilliseconds -ge $nextModeMs) {
            Send-Mode $MODE_NAV
            $nextModeMs += 100.0
        }
        Send-Chassis $DriveSpeed
        Update-Feedback
        $remaining = 20 - [int]($clock.ElapsedMilliseconds % 20)
        if ($remaining -gt 1 -and $remaining -le 20) {
            Start-Sleep -Milliseconds $remaining
        }
    }
    $clock.Stop()
    Stop-Chassis $MODE_NAV 10
    Write-Step 'NAV drive segment complete; chassis command is zero.'
}

function Hold-Mode([byte]$Mode, [double]$Seconds, [string]$Reason) {
    Write-Step ("Hold {0:F1} s: {1}" -f $Seconds, $Reason)
    $clock = [Diagnostics.Stopwatch]::StartNew()
    while ($clock.Elapsed.TotalSeconds -lt $Seconds) {
        Send-Mode $Mode
        Send-Chassis 0.0
        Update-Feedback
        Start-Sleep -Milliseconds 80
    }
}

function Move-And-Wait(
    [byte]$Mode,
    [byte]$Type,
    [double]$X,
    [double]$Y,
    [double]$Z,
    [string]$Label
) {
    Write-Step ("{0}: target type={1}, xyz=({2:F4}, {3:F4}, {4:F4}) m" -f
                $Label, $Type, $X, $Y, $Z)
    $started = Get-Date
    $nextMode = [datetime]::MinValue
    $nextTargetRetry = [datetime]::MinValue
    $stableReached = 0
    $seenMoving = $false

    while (((Get-Date) - $started).TotalSeconds -lt $ArmTimeoutSeconds) {
        if ((Get-Date) -ge $nextMode) {
            Send-Mode $Mode
            $nextMode = (Get-Date).AddMilliseconds(100)
        }
        if ((Get-Date) -ge $nextTargetRetry) {
            Send-ArmTarget $Type $X $Y $Z
            $nextTargetRetry = (Get-Date).AddSeconds(1)
        }

        Update-Feedback
        if ($script:ArmState -eq $ARM_ERROR) {
            throw "$Label failed: MCU reports ARM_ERROR."
        }
        if ($script:ArmState -eq $ARM_MOVING) { $seenMoving = $true }

        if ($script:ArmFeedbackAt -ge $started -and $script:ArmState -eq $ARM_REACHED) {
            $dx = $script:ArmX - $X
            $dy = $script:ArmY - $Y
            $dz = $script:ArmZ - $Z
            $errorM = [Math]::Sqrt($dx * $dx + $dy * $dy + $dz * $dz)
            if ($errorM -le $PositionToleranceM) {
                $stableReached++
                if ($stableReached -ge 3) {
                    Write-Step ("{0} reached: feedback=({1:F4}, {2:F4}, {3:F4}) m, error={4:F4} m, moving_seen={5}" -f
                                $Label, $script:ArmX, $script:ArmY, $script:ArmZ, $errorM, $seenMoving)
                    return
                }
            } else {
                $stableReached = 0
            }
        } else {
            $stableReached = 0
        }
    }

    throw ("{0} timed out after {1:F1} s; last state={2}, xyz=({3:F4}, {4:F4}, {5:F4})" -f
           $Label, $ArmTimeoutSeconds, $script:ArmState,
           $script:ArmX, $script:ArmY, $script:ArmZ)
}

function Run-Mission {
    if (-not $ResumeAfterArmBox1Placed) {
        Set-ModeConfirmed $MODE_NAV 'NAV'
        Drive-Distance $script:Mission.FirstNavDistanceM

        Set-ModeConfirmed $MODE_ARM 'ARM'

        Send-MainPump $true
        Move-And-Wait $MODE_ARM $TARGET_GRASP $script:Mission.ArmGrasp1[0] $script:Mission.ArmGrasp1[1] $script:Mission.ArmGrasp1[2] 'ARM box 1: ground grasp'
        Send-AuxPump $AUX_PA8 $true 'PA8 / slot 1'
        Move-And-Wait $MODE_ARM $TARGET_PLACE $script:Mission.ArmPlace1[0] $script:Mission.ArmPlace1[1] $script:Mission.ArmPlace1[2] 'ARM box 1: body placement'
    } else {
        Set-ModeConfirmed $MODE_ARM 'ARM (resume checkpoint)'
        Write-Step 'Resuming after verified ARM box 1 body placement.'
    }
    Send-MainPump $false
    Hold-Mode $MODE_ARM 3.0 'box 1 transfer complete; wait for second observation sequence'

    Send-MainPump $true
    Move-And-Wait $MODE_ARM $TARGET_GRASP $script:Mission.ArmGrasp2[0] $script:Mission.ArmGrasp2[1] $script:Mission.ArmGrasp2[2] 'ARM box 2: ground grasp'
    Send-AuxPump $AUX_PC8 $true 'PC8 / slot 2'
    Move-And-Wait $MODE_ARM $TARGET_PLACE $script:Mission.ArmPlace2[0] $script:Mission.ArmPlace2[1] $script:Mission.ArmPlace2[2] 'ARM box 2: body placement'
    Send-MainPump $false

    Set-ModeConfirmed $MODE_NAV 'NAV'
    Hold-Mode $MODE_NAV 5.0 'boxes held by PA8 and PC8'
    Drive-Distance $script:Mission.SecondNavDistanceM

    Set-ModeConfirmed $MODE_PLACE 'PLACE / REAR_PLACE'
    Send-AuxPump $AUX_PA8 $false 'PA8 / slot 1'
    Send-AuxPump $AUX_PC8 $false 'PC8 / slot 2'

    Send-MainPump $true
    Move-And-Wait $MODE_PLACE $TARGET_GRASP $script:Mission.PlaceGrasp1[0] $script:Mission.PlaceGrasp1[1] $script:Mission.PlaceGrasp1[2] 'PLACE box 1: body grasp'
    Move-And-Wait $MODE_PLACE $TARGET_PLACE $script:Mission.PlacePlace1[0] $script:Mission.PlacePlace1[1] $script:Mission.PlacePlace1[2] 'PLACE box 1: ground placement'
    Send-MainPump $false
    Hold-Mode $MODE_PLACE 3.0 'firmware place-cycle hold before box 2'

    Send-MainPump $true
    Move-And-Wait $MODE_PLACE $TARGET_GRASP $script:Mission.PlaceGrasp2[0] $script:Mission.PlaceGrasp2[1] $script:Mission.PlaceGrasp2[2] 'PLACE box 2: body grasp'
    Move-And-Wait $MODE_PLACE $TARGET_PLACE $script:Mission.PlacePlace2[0] $script:Mission.PlacePlace2[1] $script:Mission.PlacePlace2[2] 'PLACE box 2: ground placement'
    Send-MainPump $false
    Hold-Mode $MODE_PLACE 0.5 'final command settling'

    Write-Step 'MISSION COMPLETE'
}

if (-not $Execute -and -not $ValidateOnly) {
    Write-Output 'Execution guard active. Re-run with -Execute to control the robot.'
    exit 0
}

if ($ValidateOnly) {
    Load-MissionConfig
    Write-Output 'MISSION_CONFIG_VALID'
    exit 0
}

try {
    Load-MissionConfig
    try {
        $script:Serial = New-OpenedSerial
    } catch {
        Write-Step ("Initial serial open failed: {0}; waiting for re-enumeration..." -f $_.Exception.Message)
        Reconnect-Serial
    }
    $script:Serial.DiscardInBuffer()
    Write-Step "Connected to $Port at $Baud baud."
    Run-Mission
}
catch {
    $script:MissionFailed = $true
    Write-Output ("MISSION ABORTED: {0}" -f $_.Exception.Message)
    Write-Output 'Chassis will be commanded to zero. Pump outputs are preserved to avoid dropping a held box.'
}
finally {
    if ($null -ne $script:Serial -and $script:Serial.IsOpen) {
        try {
            $modeForStop = if ($script:FeedbackMode -ge 0) { [byte]$script:FeedbackMode } else { $MODE_NAV }
            Stop-Chassis $modeForStop 10
        } catch {
            Write-Output ("Final zero-speed send failed: {0}" -f $_.Exception.Message)
        }
        try {
            $script:Serial.Close()
            Write-Step "Closed $Port."
        } catch {
            Write-Output ("Serial close reported: {0}" -f $_.Exception.Message)
        }
    }
}

if ($script:MissionFailed) { exit 1 }
