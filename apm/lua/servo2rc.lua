-- Script to set RC from servo 

GPS_servo_channel_param = param:get("SCR_USER1")
Function_RCIN_start_from = 50

SERVO_FUNCTION = {
    GPS = GPS_servo_channel_param + Function_RCIN_start_from, -- RCINxx
}

RC_OPTIONS = {
    GPS_DISABLE = 65,
}

-- Timer callback interval in milliseconds
TIMER_INTERVAL_MS = 1000

local function update()

    local pwm_gps_disable = SRV_Channels:get_output_pwm(SERVO_FUNCTION.GPS)

    local rc_gps_disable = rc:find_channel_for_option(RC_OPTIONS.GPS_DISABLE)

    if pwm_gps_disable then
        rc_gps_disable:set_override(pwm_gps_disable)
    end

    -- Call the function periodically
    return update, TIMER_INTERVAL_MS
end

-- Call the function firs time
gcs:send_text(4, "Script servo to rc-channel is running")

return update()
