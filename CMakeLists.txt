cmake_minimum_required(VERSION 3.16)
project(RocketVisualizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Mac環境を想定し、Qt6 (またはQt5) を検索
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui 3DCore 3DRender 3DExtras SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui 3DCore 3DRender 3DExtras SerialPort)

add_executable(visualizer main_visualizer.cpp)

# attitude_math.hppが同じディレクトリにあるため、カレントディレクトリをインクルードパスに追加
target_include_directories(visualizer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Qtライブラリのリンク
target_link_libraries(visualizer PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::3DCore
    Qt${QT_VERSION_MAJOR}::3DRender
    Qt${QT_VERSION_MAJOR}::3DExtras
    Qt${QT_VERSION_MAJOR}::SerialPort
)
