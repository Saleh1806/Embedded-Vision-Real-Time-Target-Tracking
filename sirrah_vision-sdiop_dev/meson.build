project('Recup_Image_RAW', 'cpp')

# Build the simple RAW capture executable
simple_cam = executable('Recup_Image_RAW',
    'Recup_Image_RAW.cpp',
    dependencies: dependency('libcamera'))

# Build the RAW capture with config file support
config_cam = executable('RawCaptureWithConfig',
    'RawCaptureWithConfig.cpp',
    dependencies: dependency('libcamera'))
