headers = [
    "dronecan.sensors.hygrometer.Hygrometer.h",
    "dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes.h",
    "dronecan.sensors.rc.RCInput.h",
    "dronecan.sensors.rpm.RPM.h",
    "uavcan.equipment.actuator.ArrayCommand.h",
    "uavcan.equipment.actuator.Command.h",
    "uavcan.equipment.actuator.Status.h",
    "uavcan.equipment.ahrs.MagneticFieldStrength.h",
    "uavcan.equipment.ahrs.MagneticFieldStrength2.h",
    "uavcan.equipment.ahrs.RawIMU.h",
    "uavcan.equipment.ahrs.Solution.h",
    "uavcan.equipment.air_data.AngleOfAttack.h",
    "uavcan.equipment.air_data.IndicatedAirspeed.h",
    "uavcan.equipment.air_data.RawAirData.h",
    "uavcan.equipment.air_data.Sideslip.h",
    "uavcan.equipment.air_data.StaticPressure.h",
    "uavcan.equipment.air_data.StaticTemperature.h",
    "uavcan.equipment.air_data.TrueAirspeed.h",
    "uavcan.equipment.camera_gimbal.AngularCommand.h",
    "uavcan.equipment.camera_gimbal.GEOPOICommand.h",
    "uavcan.equipment.camera_gimbal.Mode.h",
    "uavcan.equipment.camera_gimbal.Status.h",
    "uavcan.equipment.device.Temperature.h",
    "uavcan.equipment.esc.RawCommand.h",
    "uavcan.equipment.esc.RPMCommand.h",
    "uavcan.equipment.esc.Status.h",
    "uavcan.equipment.esc.StatusExtended.h",
    "uavcan.equipment.gnss.Auxiliary.h",
    "uavcan.equipment.gnss.ECEFPositionVelocity.h",
    "uavcan.equipment.gnss.Fix.h",
    "uavcan.equipment.gnss.Fix2.h",
    "uavcan.equipment.gnss.RTCMStream.h",
    "uavcan.equipment.hardpoint.Command.h",
    "uavcan.equipment.hardpoint.Status.h",
    "uavcan.equipment.ice.FuelTankStatus.h",
    "uavcan.equipment.ice.reciprocating.CylinderStatus.h",
    "uavcan.equipment.ice.reciprocating.Status.h",
    "uavcan.equipment.indication.BeepCommand.h",
    "uavcan.equipment.indication.LightsCommand.h",
    "uavcan.equipment.indication.RGB565.h",
    "uavcan.equipment.indication.SingleLightCommand.h",
    "uavcan.equipment.power.BatteryInfo.h",
    "uavcan.equipment.power.CircuitStatus.h",
    "uavcan.equipment.power.PrimaryPowerSupplyStatus.h",
    "uavcan.equipment.range_sensor.Measurement.h",
    "uavcan.equipment.safety.ArmingStatus.h",
]

def to_macro(header):
    name = header[:-2]  # strip ".h"
    parts = name.split('.')
    msgtype = "_".join(parts)
    encfunc = f"{msgtype}_encode"
    prefix = "_".join([p.upper() for p in parts])
    id_macro = f"{prefix}_ID"
    sig_macro = f"{prefix}_SIGNATURE"
    max_macro = f"{prefix}_MAX_SIZE"
    return msgtype, encfunc, id_macro, sig_macro, max_macro

print("// Auto-generated REGISTER_UAVCAN_TYPE calls:\n")
for hdr in headers:
    msgtype, encfunc, id_macro, sig_macro, max_macro = to_macro(hdr)
    print(f"REGISTER_UAVCAN_TYPE({msgtype}, {encfunc}, {id_macro}, {sig_macro}, {max_macro});")
