# meson.build
# for SIRRAH CAMERA (full project)
# -----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# -----------------------------------------------------------------------------
#
#  Use :
#  step 1
#    $ ./build.sh
#  step 2
#    $ ./run_sirrah_demo.sh (run only sirrah_demo)
#    $ ./run_test.sh (run tests with google test)
#  Big clean to restart  : $ ./clean.sh
#
# -----------------------------------------------------------------------------

project('sirrah_camera', ['cpp'], version: '0.1.0', default_options: ['cpp_std=c++17'])

enable_tests = get_option('enable_tests')
enable_camera_acquisition = get_option('enable_camera_acquisition')

thread_dep = dependency('threads')

angle_inc = include_directories('Computing_Angle/include')

sirrah_angles = shared_library(
  'sirrah_angles',
  'Computing_Angle/src/sra_angle_calculator.cpp',
  'Computing_Angle/src/sra_angle_sender_tcp.cpp',
  include_directories: angle_inc,
  dependencies: thread_dep,
  install: true,
)

executable(
  'sirrah_demo',
  'Computing_Angle/app/main.cpp',
  include_directories: angle_inc,
  link_with: sirrah_angles,
  dependencies: thread_dep,
  install: true,
  install_rpath: '/usr/lib',
)

if enable_camera_acquisition
  fs = import('fs')
  cpp = meson.get_compiler('cpp')
  sysroot_dir = '/home/saliou/rpi5_cross/sysroot'

  libcamera_dep = cpp.find_library('camera', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libcamera_base_dep = cpp.find_library('camera-base', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libpisp_dep = cpp.find_library('pisp', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libgnutls_dep = cpp.find_library('gnutls', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  liblttng_dep = cpp.find_library('lttng-ust', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libudev_dep = cpp.find_library('udev', dirs: [sysroot_dir + '/lib/aarch64-linux-gnu'], required: false)
  libyaml_dep = cpp.find_library('yaml-0', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libdw_dep = cpp.find_library('dw', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)
  libunwind_dep = cpp.find_library('unwind', dirs: [sysroot_dir + '/usr/lib/aarch64-linux-gnu'], required: false)

  cam_deps = [
    libcamera_dep,
    libcamera_base_dep,
    libpisp_dep,
    libgnutls_dep,
    liblttng_dep,
    libudev_dep,
    libyaml_dep,
    libdw_dep,
    libunwind_dep,
  ]

  cam_inc = include_directories(sysroot_dir + '/usr/include')

  if libcamera_dep.found() and libcamera_base_dep.found()
    executable(
      'raw_capture',
      'Camera_Acquisition/RawCapture.cpp',
      dependencies: cam_deps + [thread_dep],
      include_directories: cam_inc,
      link_args: [
        '-Wl,-rpath-link=' + sysroot_dir + '/usr/lib/aarch64-linux-gnu',
        '-Wl,-rpath-link=' + sysroot_dir + '/lib/aarch64-linux-gnu',
      ],
      build_rpath: sysroot_dir + '/usr/lib/aarch64-linux-gnu:' + sysroot_dir + '/lib/aarch64-linux-gnu',
      install: true,
    )
  else
    message('libcamera not found: raw_capture will not be built')
  endif
endif

if enable_tests
  gtest_dep = dependency('gtest', required: true)
  gtest_main_dep = dependency('gtest_main', required: true)
  gtest_inc = include_directories('/usr/include')

  test_exe = executable(
    'sirrah_angle_tests',
    'Computing_Angle/tests/sra_angle_calculator_test.cpp',
    include_directories: [angle_inc, gtest_inc],
    link_with: sirrah_angles,
    dependencies: [gtest_dep, gtest_main_dep, thread_dep],
  )

  test('angle_computation', test_exe)
endif
