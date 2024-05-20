
# tools
option(use_hwcomponents_tools "Use tools" ON)
option(use_prebuilt_hwcomponents_tools "Use prebuilt tools" ON)
option(use_hwcomponents_custom_define_assert "Use custom define assert" ON)
# set(use_hwcomponents_custom_define_assert ON CACHE INTERNAL "Use custom define assert")

# algorithms
option(use_hwcomponents_algorithms_controller "Use algorithms controller" ON)
option(use_prebuilt_hwcomponents_algorithms_controller "Use prebuilt algorithms controller" ON)

option(use_hwcomponents_algorithms_ik_solver_chassis "Use algorithms ik_solver" ON)
option(use_prebuilt_hwcomponents_algorithms_ik_solver_chassis "Use prebuilt algorithms ik_solver" ON)
option(CHASSIS_IKSOLVER_USE_ARM_MATH "chassis_ik_solver use math function from arm_math.h" ON)

option(use_hwcomponents_algorithms_ahrs "Use AHRS" ON)
option(use_prebuilt_hwcomponents_algorithms_ahrs "Use prebuilt AHRS" ON)

option(use_hwcomponents_algorithms_filter "Use filter" ON)
option(use_prebuilt_hwcomponents_algorithms_filter "Use prebuilt filter" ON)

option(use_hwcomponents_algorithms_observer "Use observer" ON)
option(use_prebuilt_hwcomponents_algorithms_observer "Use prebuilt observer" ON)

option(use_hwcomponents_algorithms_fsm "Use fsm" ON)
option(use_prebuilt_hwcomponents_algorithms_fsm "Use prebuilt fsm" ON)

# devices
option(use_hwcomponents_devices_referee "Use referee device" ON)
option(use_prebuilt_hwcomponents_devices_referee "Use prebuilt referee" ON)

option(use_hwcomponents_devices_motor "Use motor device" ON)
option(use_prebuilt_hwcomponents_devices_motor "Use prebuilt motor" ON)

option(use_hwcomponents_devices_laser "Use laser device" ON)
option(use_prebuilt_hwcomponents_devices_laser "Use prebuilt laser" ON)

option(use_hwcomponents_devices_servo "Use servo device" ON)
option(use_prebuilt_hwcomponents_devices_servo "Use prebuilt servo" ON)

option(use_hwcomponents_devices_buzzer "Use buzzer device" ON)
option(use_prebuilt_hwcomponents_devices_buzzer "Use prebuilt buzzer" ON)

option(use_hwcomponents_devices_remote_control "Use remote control device" ON)
option(use_prebuilt_hwcomponents_devices_remote_control "Use prebuilt remote control" ON)

option(use_hwcomponents_devices_imu "Use imu device" ON)
option(use_prebuilt_hwcomponents_devices_imu "Use prebuilt imu" ON)

# dsp
option(use_hwcomponents_bsp_tick "Use bsp tick" ON)
option(use_prebuilt_hwcomponents_bsp_tick "Use prebuilt bsp tick" ON)

option(use_hwcomponents_bsp_stm32_hal "Use bsp stm32 hal" ON)

# Third_Party
option(use_hwcomponents_third_party_dsp "Use third party: dsp" ON)
option(use_prebuilt_hwcomponents_third_party_dsp "Use prebuilt third party: dsp" ON)



