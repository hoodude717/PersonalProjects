idf_component_register(SRCS main.c players/player.c players/playerFire.c players/playerWater.c gameControl/playerMovement.c gameControl/playerJump.c gameControl/gameControl.c gameControl/platforms.c gameControl/switches.c gameControl/elevator.c gameControl/door.c gameControl/obstacles.c
                       INCLUDE_DIRS . players gameControl
                       PRIV_REQUIRES esp_timer driver config lcd pin joy cursor sound c24k_8b)
# PRIV_REQUIRES ...
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
