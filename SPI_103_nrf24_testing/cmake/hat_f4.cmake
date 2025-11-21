message(CHECK_START "Include HAT_F4")

include(FetchContent)

FetchContent_Declare(hat_repo
  GIT_REPOSITORY ssh://git@dev.kb-avrora.ru:2222/kuklotar/hat_f4.git
  GIT_TAG        master
)

FetchContent_MakeAvailable(hat_repo)

message(CHECK_PASS "done")
