view_sources = [
  'sfml-view.cc',
]

inc = include_directories('../loftlib')

gtkmm_dep = dependency('gtkmm-3.0')
sfml_dep = dependency('sfml-all')
gl_dep = dependency('opengl')
glu_dep = dependency('glu')
glut_dep = dependency('glut')

view = executable('view',
                  view_sources,
                  include_directories: inc,
                  dependencies: [sfml_dep, gl_dep, glu_dep, glut_dep],
                  link_with: [loftlib],
                  override_options : ['cpp_std=c++2a'],
)
