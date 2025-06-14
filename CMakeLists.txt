cmake_minimum_required(VERSION 3.10)
project(turtle_go_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź podstawowe zależności ROS 2
find_package(ament_cmake REQUIRED)

# Znajdź pakiety potrzebne do przetwarzania URDF/XACRO
# Są one wymagane, aby CMake wiedział o narzędziach do pracy z URDF/XACRO
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# --- Instalacja zasobów pakietu ---

# Zainstaluj katalog z plikami URDF/XACRO robota
# Zostaną one skopiowane do share/turtle_go_urdf/urdf
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Zainstaluj katalog z plikami konfiguracyjnymi (np. dla kontrolerów YAML)
# Zostaną one skopiowane do share/turtle_go_urdf/config
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Zainstaluj katalog z plikami launch
# Zostaną one skopiowane do share/turtle_go_urdf/launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Opcjonalnie: Jeśli masz jakieś skrypty Pythona lub inne moduły, zainstaluj je:
# find_package(ament_cmake_python REQUIRED)
# install(
#   PROGRAMS scripts/my_script.py # Dla pojedynczych skryptów
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(
#   DIRECTORY python/my_module # Dla modułów Pythona
#   DESTINATION lib/${PROJECT_NAME}
# )

# Deklaracja pakietu ament
ament_package()