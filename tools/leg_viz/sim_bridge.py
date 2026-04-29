"""ctypes wrapper around libfengmh_sim.dylib.

Loads the shared library built by `cmake --build build_host --target fengmh_sim`
and exposes Python-friendly wrappers for IK/FK and script sampling so that
tools/leg_viz/leg_viz.py can drive the same C code that runs on the MCU.
"""
from __future__ import annotations

import ctypes as C
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DYLIB_PATH = PROJECT_ROOT / "build_host" / "libfengmh_sim.dylib"

if not DYLIB_PATH.exists():
    raise RuntimeError(
        f"libfengmh_sim.dylib not found at {DYLIB_PATH}\n"
        "Build it first:\n"
        "  cmake -S App/test -B build_host\n"
        "  cmake --build build_host --target fengmh_sim"
    )

_lib = C.CDLL(str(DYLIB_PATH))


# --- struct mirrors (must match leg_params.h / leg_ik.h / script_if.h) ---

class _AngleRange(C.Structure):
    _fields_ = [("min", C.c_float), ("max", C.c_float)]


class LegDim(C.Structure):
    _fields_ = [
        ("thigh_length", C.c_float),
        ("shin_length", C.c_float),
        ("link_length", C.c_float),
        ("wheel_diameter", C.c_float),
        ("thigh_angle_range", _AngleRange),
        ("shin_angle_range", _AngleRange),
        ("thigh_mass_1", C.c_float),
        ("thigh_mass_2", C.c_float),
        ("shin_mass", C.c_float),
        ("link_mass", C.c_float),
        ("wheel_mass", C.c_float),
        ("lc_t_m_1", C.c_float),
        ("lc_t_m_2", C.c_float),
        ("lc_s_m", C.c_float),
        ("lc_l_m", C.c_float),
        ("g", C.c_float),
    ]


class IkResult(C.Structure):
    _fields_ = [("theta1", C.c_float), ("theta2", C.c_float)]


class FkResult(C.Structure):
    _fields_ = [("x", C.c_float), ("z", C.c_float)]


class LinkagePose(C.Structure):
    _fields_ = [
        ("hip_x", C.c_float),
        ("hip_z", C.c_float),
        ("knee_x", C.c_float),
        ("knee_z", C.c_float),
        ("crank_x", C.c_float),
        ("crank_z", C.c_float),
        ("lower_mount_x", C.c_float),
        ("lower_mount_z", C.c_float),
        ("foot_x", C.c_float),
        ("foot_z", C.c_float),
    ]


class GaitLegTarget(C.Structure):
    _fields_ = [
        ("hip_rad", C.c_float),
        ("knee_rad", C.c_float),
        ("wheel_rads", C.c_float),
        ("in_stance", C.c_uint8),
    ]


GAIT_LEG_NUM = 4


class GaitOutput(C.Structure):
    _fields_ = [
        ("leg", GaitLegTarget * GAIT_LEG_NUM),
        ("phase", C.c_float),
        ("tick_count", C.c_uint32),
    ]


class GaitParams(C.Structure):
    _fields_ = [
        ("body_height_m", C.c_float),
        ("step_length_m", C.c_float),
        ("step_height_m", C.c_float),
        ("period_s", C.c_float),
        ("duty", C.c_float),
        ("phase_offset", C.c_float * GAIT_LEG_NUM),
        ("touchdown_thresh", C.c_float),
    ]


class _ScriptT(C.Structure):
    _fields_ = [
        ("name", C.c_char_p),
        ("frames", C.c_void_p),
        ("n_frames", C.c_uint16),
        ("loop", C.c_uint8),
    ]


LEG_ACT_NUM = 3
LEG_ACT_HIP = 0
LEG_ACT_KNEE = 1
LEG_ACT_WHEEL = 2


class LegConfig(C.Structure):
    _fields_ = [
        ("leg", C.c_int),
        ("name", C.c_char_p),
        ("label", C.c_char_p),
        ("leg_type", C.c_int),
        ("motor", C.c_int * LEG_ACT_NUM),
        ("body_x_m", C.c_float),
        ("body_y_m", C.c_float),
        ("foot_x_dir", C.c_float),
    ]


class MotorCfg(C.Structure):
    _fields_ = [
        ("logical", C.c_int),
        ("type", C.c_int),
        ("can_bus", C.c_uint8),
        ("can_id", C.c_uint32),
        ("dir", C.c_int8),
        ("zero_offset", C.c_float),
        ("gear_ratio", C.c_float),
        ("boot_angle", C.c_float),
        ("limit_min", C.c_float),
        ("limit_max", C.c_float),
        ("name", C.c_char_p),
    ]


class HostMotorSnapshot(C.Structure):
    _fields_ = [
        ("online", C.c_uint8),
        ("angle_rad", C.c_float),
        ("velocity_rads", C.c_float),
        ("torque_nm", C.c_float),
        ("pos_target_rad", C.c_float),
        ("vel_target_rads", C.c_float),
        ("kp", C.c_float),
        ("kd", C.c_float),
        ("tau_ff", C.c_float),
        ("pos_calls", C.c_uint32),
        ("vel_calls", C.c_uint32),
    ]


class HostLegPose(C.Structure):
    _fields_ = [
        ("hip_cmd_rad", C.c_float),
        ("knee_cmd_rad", C.c_float),
        ("hip_raw_rad", C.c_float),
        ("knee_raw_rad", C.c_float),
        ("foot_body_x_m", C.c_float),
        ("foot_body_y_m", C.c_float),
        ("foot_down_z_m", C.c_float),
        ("knee_body_x_m", C.c_float),
        ("knee_body_y_m", C.c_float),
        ("knee_down_z_m", C.c_float),
        ("crank_body_x_m", C.c_float),
        ("crank_body_y_m", C.c_float),
        ("crank_down_z_m", C.c_float),
        ("lower_mount_body_x_m", C.c_float),
        ("lower_mount_body_y_m", C.c_float),
        ("lower_mount_down_z_m", C.c_float),
        ("ik_ok", C.c_uint8),
    ]


@dataclass(frozen=True)
class MotorInfo:
    logical: int
    name: str
    role: str
    type_name: str
    can_bus: int
    can_id: int
    direction: int
    zero_offset: float
    gear_ratio: float
    boot_angle: float
    limit_min: float
    limit_max: float


@dataclass(frozen=True)
class LegInfo:
    index: int
    name: str
    label: str
    leg_type: int
    leg_type_name: str
    body_x_m: float
    body_y_m: float
    foot_x_dir: float
    motors: tuple[MotorInfo, MotorInfo, MotorInfo]


# --- prototypes ---

LEG_TYPE_ORIGINAL = 0
LEG_TYPE_MIRROR = 1

_lib.leg_ik_solve.argtypes = [
    C.c_float, C.c_float, C.POINTER(LegDim), C.c_float, C.c_int, C.POINTER(IkResult)
]
_lib.leg_ik_solve.restype = C.c_int

_lib.leg_fk_solve.argtypes = [
    C.c_float, C.c_float, C.POINTER(LegDim), C.POINTER(FkResult)
]
_lib.leg_fk_solve.restype = None

_lib.leg_linkage_solve.argtypes = [
    C.c_float, C.c_float, C.POINTER(LegDim), C.POINTER(LinkagePose)
]
_lib.leg_linkage_solve.restype = None

_lib.script_builtin_find.argtypes = [C.c_char_p]
_lib.script_builtin_find.restype = C.POINTER(_ScriptT)

_lib.script_sample.argtypes = [C.POINTER(_ScriptT), C.c_float, C.POINTER(GaitOutput)]
_lib.script_sample.restype = C.c_int

_lib.leg_config_count.argtypes = []
_lib.leg_config_count.restype = C.c_uint32

_lib.leg_config_get.argtypes = [C.c_int]
_lib.leg_config_get.restype = C.POINTER(LegConfig)

_lib.leg_config_type_name.argtypes = [C.c_int]
_lib.leg_config_type_name.restype = C.c_char_p

_lib.leg_config_motor_role_name.argtypes = [C.c_int]
_lib.leg_config_motor_role_name.restype = C.c_char_p

_lib.motor_get_cfg.argtypes = [C.c_int]
_lib.motor_get_cfg.restype = C.POINTER(MotorCfg)

_lib.host_sim_init.argtypes = []
_lib.host_sim_init.restype = C.c_int

_lib.host_sim_set_stand_height.argtypes = [C.c_float]
_lib.host_sim_set_stand_height.restype = C.c_int

_lib.host_sim_apply_foot_targets.argtypes = [
    C.POINTER(C.c_float), C.POINTER(C.c_float), C.POINTER(C.c_float)
]
_lib.host_sim_apply_foot_targets.restype = C.c_int

_lib.host_sim_set_trot_params.argtypes = [C.POINTER(GaitParams)]
_lib.host_sim_set_trot_params.restype = C.c_int

_lib.host_sim_play_trot.argtypes = [C.POINTER(GaitParams), C.c_float]
_lib.host_sim_play_trot.restype = C.c_int

_lib.host_sim_play_script.argtypes = [C.c_char_p]
_lib.host_sim_play_script.restype = C.c_int

_lib.host_sim_stop_script.argtypes = []
_lib.host_sim_stop_script.restype = C.c_int

_lib.host_sim_step.argtypes = [C.c_float]
_lib.host_sim_step.restype = C.c_int

_lib.host_sim_active_gait_name.argtypes = []
_lib.host_sim_active_gait_name.restype = C.c_char_p

_lib.host_sim_get_motor_snapshot.argtypes = [C.c_int, C.POINTER(HostMotorSnapshot)]
_lib.host_sim_get_motor_snapshot.restype = C.c_int

_lib.host_sim_get_leg_pose.argtypes = [C.c_int, C.POINTER(HostLegPose)]
_lib.host_sim_get_leg_pose.restype = C.c_int

LEG_DIM_DEFAULT = LegDim.in_dll(_lib, "LEG_DIM_DEFAULT")
GAIT_PARAMS_TROT_DEFAULT = GaitParams.in_dll(_lib, "GAIT_PARAMS_TROT_DEFAULT")


MOTOR_TYPE_NAMES = {
    0: "UNKNOWN",
    1: "M3508",
    2: "GO",
    3: "DAMIAO",
}


def _cstr(p) -> str:
    if not p:
        return ""
    return p.decode("utf-8")


def motor_cfg(logical: int, role: int | None = None) -> MotorInfo:
    p = _lib.motor_get_cfg(C.c_int(logical))
    if not p:
        raise ValueError(f"no motor cfg for logical id {logical}")
    c = p.contents
    role_name = _cstr(_lib.leg_config_motor_role_name(role)) if role is not None else ""
    return MotorInfo(
        logical=c.logical,
        name=_cstr(c.name),
        role=role_name,
        type_name=MOTOR_TYPE_NAMES.get(c.type, f"type{c.type}"),
        can_bus=int(c.can_bus),
        can_id=int(c.can_id),
        direction=int(c.dir),
        zero_offset=float(c.zero_offset),
        gear_ratio=float(c.gear_ratio),
        boot_angle=float(c.boot_angle),
        limit_min=float(c.limit_min),
        limit_max=float(c.limit_max),
    )


def list_leg_configs() -> list[LegInfo]:
    legs: list[LegInfo] = []
    for i in range(int(_lib.leg_config_count())):
        p = _lib.leg_config_get(C.c_int(i))
        if not p:
            continue
        c = p.contents
        motors = tuple(motor_cfg(int(c.motor[role]), role) for role in range(LEG_ACT_NUM))
        legs.append(LegInfo(
            index=int(c.leg),
            name=_cstr(c.name),
            label=_cstr(c.label),
            leg_type=int(c.leg_type),
            leg_type_name=_cstr(_lib.leg_config_type_name(c.leg_type)),
            body_x_m=float(c.body_x_m),
            body_y_m=float(c.body_y_m),
            foot_x_dir=float(c.foot_x_dir),
            motors=motors,  # type: ignore[arg-type]
        ))
    return legs


def motor_command_angle(raw_rad: float, motor: MotorInfo) -> float:
    gear = motor.gear_ratio if motor.gear_ratio > 0.0 else 1.0
    return float(motor.direction) * float(raw_rad) * gear + float(motor.zero_offset)


# --- python-facing helpers ---

def make_dim(thigh: float | None = None, shin: float | None = None) -> LegDim:
    """Copy LEG_DIM_DEFAULT, optionally overriding thigh/shin lengths."""
    d = LegDim()
    C.memmove(C.byref(d), C.byref(LEG_DIM_DEFAULT), C.sizeof(LegDim))
    if thigh is not None:
        d.thigh_length = float(thigh)
    if shin is not None:
        d.shin_length = float(shin)
    return d


def make_gait_params(
    *,
    body_height_m: float | None = None,
    step_length_m: float | None = None,
    step_height_m: float | None = None,
    period_s: float | None = None,
    duty: float | None = None,
    phase_offset=None,
    touchdown_thresh: float | None = None,
) -> GaitParams:
    p = GaitParams()
    C.memmove(C.byref(p), C.byref(GAIT_PARAMS_TROT_DEFAULT), C.sizeof(GaitParams))
    if body_height_m is not None:
        p.body_height_m = float(body_height_m)
    if step_length_m is not None:
        p.step_length_m = float(step_length_m)
    if step_height_m is not None:
        p.step_height_m = float(step_height_m)
    if period_s is not None:
        p.period_s = float(period_s)
    if duty is not None:
        p.duty = float(duty)
    if phase_offset is not None:
        vals = list(phase_offset)
        if len(vals) != GAIT_LEG_NUM:
            raise ValueError(f"phase_offset must have {GAIT_LEG_NUM} values")
        for i, v in enumerate(vals):
            p.phase_offset[i] = float(v)
    if touchdown_thresh is not None:
        p.touchdown_thresh = float(touchdown_thresh)
    return p


def solve_ik(x: float, z: float, dim: LegDim | None = None,
             height: float = 0.0, leg_type: int = LEG_TYPE_ORIGINAL):
    """Foot (x,z) m -> (theta1, theta2) rad. Returns None if out of workspace."""
    if dim is None:
        dim = LEG_DIM_DEFAULT
    out = IkResult()
    rc = _lib.leg_ik_solve(
        C.c_float(x), C.c_float(z),
        C.byref(dim), C.c_float(height), C.c_int(leg_type),
        C.byref(out),
    )
    if rc != 0:
        return None
    return out.theta1, out.theta2


def solve_leg_ik(leg: LegInfo, x: float, z: float, dim: LegDim | None = None,
                 height: float = 0.0):
    raw = solve_ik(x, z, dim=dim, height=height, leg_type=leg.leg_type)
    if raw is None:
        return None
    hip_raw, knee_raw = raw
    hip_cmd = motor_command_angle(hip_raw, leg.motors[LEG_ACT_HIP])
    knee_cmd = motor_command_angle(knee_raw, leg.motors[LEG_ACT_KNEE])
    return hip_raw, knee_raw, hip_cmd, knee_cmd


def solve_fk(theta1: float, theta2: float, dim: LegDim | None = None):
    """(theta1, theta2) rad -> foot (x, z) m."""
    if dim is None:
        dim = LEG_DIM_DEFAULT
    out = FkResult()
    _lib.leg_fk_solve(
        C.c_float(theta1), C.c_float(theta2),
        C.byref(dim), C.byref(out),
    )
    return out.x, out.z


def solve_linkage(theta1: float, theta2: float, dim: LegDim | None = None) -> LinkagePose:
    """(theta1, theta2) rad -> real linkage points in the hip-local X/Z plane."""
    if dim is None:
        dim = LEG_DIM_DEFAULT
    out = LinkagePose()
    _lib.leg_linkage_solve(
        C.c_float(theta1), C.c_float(theta2),
        C.byref(dim), C.byref(out),
    )
    return out


BUILTIN_SCRIPT_NAMES = ("stand_hold", "wave_up_down", "trot_step")


def list_builtin_scripts() -> list[str]:
    """Return names that script_builtin_find resolves successfully."""
    found = []
    for n in BUILTIN_SCRIPT_NAMES:
        if _lib.script_builtin_find(n.encode("utf-8")):
            found.append(n)
    return found


def sample_script(name: str, t_s: float):
    """Call script_sample(name, t_s); return GaitOutput or None on failure."""
    sp = _lib.script_builtin_find(name.encode("utf-8"))
    if not sp:
        return None
    out = GaitOutput()
    rc = _lib.script_sample(sp, C.c_float(t_s), C.byref(out))
    if rc != 0:
        return None
    return out


def script_duration(name: str) -> float:
    """Read the last keyframe's t_s for animation length. 0 if not found."""
    sp = _lib.script_builtin_find(name.encode("utf-8"))
    if not sp:
        return 0.0
    s = sp.contents
    if s.n_frames == 0 or not s.frames:
        return 0.0
    # script_keyframe_t = { float t_s; gait_leg_target_t leg[4]; }
    # gait_leg_target_t is 4*float + 1*uint8 with 3 pad = 16 bytes (default align)
    # struct size = 4 + 4*16 = 68 bytes if no extra trailing pad; safer to compute
    # via the sizeof alignment of float -> use a Structure mirror.
    class _Kf(C.Structure):
        _fields_ = [("t_s", C.c_float), ("leg", GaitLegTarget * GAIT_LEG_NUM)]
    arr_t = _Kf * s.n_frames
    arr = arr_t.from_address(s.frames)
    return float(arr[s.n_frames - 1].t_s)


class HostSystemSim:
    """Terminal executor backed by the real C chassis/leg/motor stack."""

    def __init__(self):
        rc = _lib.host_sim_init()
        if rc != 0:
            raise RuntimeError(f"host_sim_init failed: {rc}")

    def set_stand_height(self, height_m: float):
        rc = _lib.host_sim_set_stand_height(C.c_float(height_m))
        if rc != 0:
            raise RuntimeError(f"host_sim_set_stand_height failed: {rc}")

    def apply_foot_targets(self, dx_m, dz_m, wheel_rads=None):
        if wheel_rads is None:
            wheel_rads = [0.0 for _ in range(GAIT_LEG_NUM)]
        dx_arr = (C.c_float * GAIT_LEG_NUM)(*[float(v) for v in dx_m])
        dz_arr = (C.c_float * GAIT_LEG_NUM)(*[float(v) for v in dz_m])
        wh_arr = (C.c_float * GAIT_LEG_NUM)(*[float(v) for v in wheel_rads])
        rc = _lib.host_sim_apply_foot_targets(dx_arr, dz_arr, wh_arr)
        if rc != 0:
            raise RuntimeError(f"host_sim_apply_foot_targets failed: {rc}")

    def set_trot_params(self, params: GaitParams):
        rc = _lib.host_sim_set_trot_params(C.byref(params))
        if rc != 0:
            raise RuntimeError(f"host_sim_set_trot_params failed: {rc}")

    def play_trot(self, params: GaitParams, blend_dur_s: float = 0.0):
        rc = _lib.host_sim_play_trot(C.byref(params), C.c_float(blend_dur_s))
        if rc != 0:
            raise RuntimeError(f"host_sim_play_trot failed: {rc}")

    def play_script(self, name: str):
        rc = _lib.host_sim_play_script(name.encode("utf-8"))
        if rc != 0:
            raise RuntimeError(f"host_sim_play_script({name!r}) failed: {rc}")

    def stop_script(self):
        _lib.host_sim_stop_script()

    def step(self, dt_s: float):
        rc = _lib.host_sim_step(C.c_float(dt_s))
        if rc != 0:
            raise RuntimeError(f"host_sim_step failed: {rc}")

    def active_gait_name(self) -> str:
        return _cstr(_lib.host_sim_active_gait_name())

    def motor_snapshot(self, logical_id: int) -> HostMotorSnapshot:
        out = HostMotorSnapshot()
        rc = _lib.host_sim_get_motor_snapshot(C.c_int(logical_id), C.byref(out))
        if rc != 0:
            raise RuntimeError(f"host_sim_get_motor_snapshot({logical_id}) failed: {rc}")
        return out

    def leg_pose(self, leg_id: int) -> HostLegPose:
        out = HostLegPose()
        rc = _lib.host_sim_get_leg_pose(C.c_int(leg_id), C.byref(out))
        if rc != 0:
            raise RuntimeError(f"host_sim_get_leg_pose({leg_id}) failed: {rc}")
        return out
