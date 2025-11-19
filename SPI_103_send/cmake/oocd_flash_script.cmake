string(CONCAT OOCD_PROG ${PROG} .cfg)
string(CONCAT OOCD_TARGET ${PROG_TARGET} .cfg)

set(OOCD_FLASH_SCRIPT
     "MESSAGE(STATUS \"Flashing firmware\")
     execute_process (
          COMMAND openocd
               -f interface/${OOCD_PROG}
               -f target/${OOCD_TARGET}
               -c \"init\" 
               -c \"reset halt\"
               -c \"program ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}\" 
               -c \"reset\"
               -c \"shutdown\"
     )
     MESSAGE(STATUS \"Flashing complete\")
     "
)
