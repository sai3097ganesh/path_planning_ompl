# Minimal shim to satisfy OMPL's python detection without requiring distutils.
# We do not build Python bindings, so we simply report Python not found.
set(PYTHON_FOUND FALSE)
set(Python_FOUND FALSE)
set(PYTHON_EXECUTABLE "")

macro(find_boost_python)
  # no-op: Python is not used
endmacro()

macro(find_python_module)
  # no-op
endmacro()

macro(install_python)
  # no-op
endmacro()
