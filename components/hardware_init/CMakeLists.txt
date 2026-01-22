idf_component_register(
    SRCS "src/hardware_init.cc"
    INCLUDE_DIRS "include"                # 标准组件路径
        
    REQUIRES 
        driver
        espressif__esp_simplefoc
        espressif2022__bmi270
)