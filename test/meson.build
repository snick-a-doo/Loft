loft_test_sources = [
  'test.cc',
  'test-body.cc',
]

inc = include_directories('.', '../loftlib')
loft_test = executable('test-loft',
                       loft_test_sources,
                       include_directories: inc,
                       link_with: [loftlib],
                       override_options : ['cpp_std=c++2a'])
test('loft test', loft_test)
