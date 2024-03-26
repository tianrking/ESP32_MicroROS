idf_component_register(
    SRCS 
        "main.c"                                # Main entry point
        "wit_c_sdk.c"                           # Wit.ai C SDK
        "wit_task.c"                            # Wit.ai task
        "../example/microros.c"                 # Micro-ROS 
        "../example/mcpwm_motor_control.c"      # Motor control using MCPWM
        "../example/pcnt_rotary_encoder.c"      # Rotary encoder using PCNT
        "../network_3rd/http_handle.c"
        "../network_3rd/websocket_handle.c"
        "../network_3rd/mqtt_handle.c"
        "../ble_3rd/ble_handle.c"
    INCLUDE_DIRS 
        ""
        "../ble_3rd/"
        "../example/"
        "../network_3rd/"
)
