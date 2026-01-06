
idf_component_register(
    SRCS
        "main.cc"
        "main_functions.cc"
        "detection_responder.cc"
        "image_provider.cc"
        "model_settings.cc"
        "person_detect_model_data.cc"
        "app_camera_esp.c"
        "esp_cli.c"
        "custom_images.c"
        "led_strip_encoder.c"
    INCLUDE_DIRS "."
)