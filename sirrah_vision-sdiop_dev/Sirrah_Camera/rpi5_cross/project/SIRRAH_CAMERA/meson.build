project('RawCaptureWithConfig', 'cpp',
  version: '1.0.0',
  default_options: ['cpp_std=c++17']
)

# ================================
# 🔹 Configuration du sysroot
# ================================
sysroot_dir = '/home/saliou/rpi5_cross/sysroot'

cpp = meson.get_compiler('cpp')

# ================================
# 🔹 Dépendances principales
# ================================
libcamera_dep       = cpp.find_library('camera', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libcamera_base_dep  = cpp.find_library('camera-base', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libpisp_dep         = cpp.find_library('pisp', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libgnutls_dep       = cpp.find_library('gnutls', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
liblttng_dep        = cpp.find_library('lttng-ust', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libudev_dep         = cpp.find_library('udev', dirs : [sysroot_dir + '/lib/aarch64-linux-gnu'])
libyaml_dep         = cpp.find_library('yaml-0', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libdw_dep           = cpp.find_library('dw', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])
libunwind_dep       = cpp.find_library('unwind', dirs : [sysroot_dir + '/usr/lib/aarch64-linux-gnu'])

common_deps = [
  libcamera_dep,
  libcamera_base_dep,
  libpisp_dep,
  libgnutls_dep,
  liblttng_dep,
  libudev_dep,
  libyaml_dep,
  libdw_dep,
  libunwind_dep
]

common_link_args = [
  '-Wl,-rpath-link=' + sysroot_dir + '/usr/lib/aarch64-linux-gnu',
  '-Wl,-rpath-link=' + sysroot_dir + '/lib/aarch64-linux-gnu'
]
# ================================
# 🔹 Compilation du binaire principal
# ================================
exe = executable('RawCaptureWithConfig',
  'ConfigParam.cpp',
  include_directories: include_directories(sysroot_dir + '/usr/include'),
  dependencies: common_deps,
  link_args: common_link_args,
  build_rpath: sysroot_dir + '/usr/lib/aarch64-linux-gnu:' + sysroot_dir + '/lib/aarch64-linux-gnu',
  install: true,
  install_dir: get_option('bindir')
)

# ================================
# 🔹 Installation du fichier de configuration
# ================================
install_data('data/configParam.ini',
  install_dir: get_option('datadir') / 'rawcapture',
  rename: 'configParam.ini'
)

# ================================
# 🔹 Script post-installation (optionnel)
# ================================
meson.add_install_script('scripts/post_install.sh')
