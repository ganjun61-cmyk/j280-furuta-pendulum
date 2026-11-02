#!/usr/bin/env python3
"""J280 v4 profiles and bounded host recording/control; no host START command.

The profile's power label is supplied by a person, not measured by this tool.
Generated profiles are isolated logic inputs using provisional design A.
Neither their label nor successful numeric validation qualifies powered use.
"""

import json
import math
from pathlib import Path

import j280_balance_math as balance_math
import j280_balance_protocol as protocol


def build_isolated_config(upright_code, config_id=1) -> dict:
    """Build an isolated profile; round caller-supplied upright Q16 half away from zero."""
    if type(upright_code) not in (int, float) or not math.isfinite(upright_code):
        raise ValueError("upright_code must be an explicit finite number, not a bool")
    scaled = upright_code * 65536
    if not math.isfinite(scaled):
        raise ValueError("upright_code cannot be represented as Q16")
    upright_q16 = int(math.copysign(math.floor(abs(scaled) + 0.5), scaled))
    values = protocol.validate_values(dict(
        config_id=config_id, algorithm_id=1, run_ms=1000, keepalive_ms=200,
        travel_limit_count=100, speed_limit_count_s=5000, adc_speed_limit_code_s=10000,
        max_compare=1000, hold_ms=200, wait_ms=5000, upright_zero_q16=upright_q16,
        enter_code=8, exit_code=45, comp_compare=175, blend_compare=25,
        comp_gain_q20=7 << 20, coefficients=balance_math.design_coefficients(),
        sample_period_us=2000,
    ))
    sources = {key: "v4 isolated logic default; not a powered qualification"
               for key in protocol.VALUE_FIELDS}
    sources.update(
        config_id="caller-supplied config_id (default 1)",
        upright_zero_q16="explicit caller upright_code, nearest Q16 half away from zero; calibration not inferred",
        coefficients="j280_balance_math.design_coefficients() default design A, provisional identified model",
    )
    return dict(schema="j280-balance-config-v1", power="isolated",
                purpose="Offline isolated logic configuration. The power label is not a measurement; "
                        "design A and caller upright zero require separate physical qualification.",
                values=values, sources=sources)


SWING_SCHEMA = "j280-swing-config-v1"


def build_swing_config(base, params, config_id, exit_code=80, **swing_options) -> dict:
    """v5 profile: a validated v4 profile's CONFIG plus a SWING block compiled from float params."""
    import j280_balance_design as design
    import j280_swing_math as swing_math
    values = dict(base["values"], config_id=config_id, exit_code=exit_code)
    swing = swing_math.compile_params(params, config_id=config_id, max_compare=values["max_compare"],
                                      w0=design.PENDULUM["w0"], count_rad=design.COUNT_RAD,
                                      **swing_options)
    sources = dict(base["sources"], config_id=f"swing profile {config_id}",
                   exit_code=f"{exit_code} codes: room for a catch at {params['catch_deg']} deg")
    return dict(schema=SWING_SCHEMA, power=base["power"],
                purpose=f"Swing-up v5 from {base['values']['config_id']} with {json.dumps(params)}",
                values=values, sources=sources, swing=swing, swing_params=params)


def load_config(path) -> dict:
    """Validate a profile and its per-field provenance, without measuring power."""
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("configuration must be a JSON object")
    if data.get("schema") not in ("j280-balance-config-v1", SWING_SCHEMA):
        raise ValueError(f"schema must be j280-balance-config-v1 or {SWING_SCHEMA}")
    if data.get("power") not in ("isolated", "powered"):
        raise ValueError("power must be isolated or powered (a label, not measured power)")
    if not isinstance(data.get("purpose"), str) or not data["purpose"].strip():
        raise ValueError("a non-empty purpose is required")
    v6 = "traj" in data
    if v6:
        import j280_traj as traj_math
        import j280_v6_protocol as protocol6
        if "backlash" in data:
            raise ValueError("v6 trajectory profiles have no backlash fields")
        data["traj"] = traj_math.validate(data["traj"])
    if data["schema"] == SWING_SCHEMA:
        import j280_swing_math as swing_math
        import j280_swing_protocol as protocol5
        data["values"] = (protocol6 if v6 else protocol5).validate_values(data.get("values"))
        data["swing"] = swing_math.validate(data.get("swing"))
        if data["swing"]["config_id"] != data["values"]["config_id"]:
            raise ValueError("swing config_id must equal the CONFIG config_id")
    else:
        data["values"] = (protocol6 if v6 else protocol).validate_values(data.get("values"))
    if "backlash" in data or "gains2" in data:
        import j280_backlash as backlash_math
        import j280_swing_protocol as protocol5
        if "backlash" in data:
            data["backlash"] = backlash_math.validate(data["backlash"])
        if "gains2" in data:
            g = protocol5.validate_gains2(data["gains2"])
            data["gains2"] = dict(data["gains2"], switch_ms=g["switch_ms"], coefficients=g["coefficients"])
        if data["schema"] != SWING_SCHEMA and not v6:
            protocol5.validate_values(data["values"])
    sources = data.get("sources")
    if (not isinstance(sources, dict) or set(sources) != set(protocol.VALUE_FIELDS)
            or any(not isinstance(source, str) or not source.strip() for source in sources.values())):
        raise ValueError("every value requires exactly one non-empty source")
    return data
