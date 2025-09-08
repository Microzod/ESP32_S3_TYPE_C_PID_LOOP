idf_component_register(
    SRCS 
        "src/MCPWM_DUAL_PWM.cpp"
        "src/L298N.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)