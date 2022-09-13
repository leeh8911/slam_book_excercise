set(UTILS Utils)
set(PLOT_TRAJECTORY PlotTrajectory)
set(TRAJECTORY_ERROR TrajectoryError)

add_subdirectory(utils)
add_subdirectory(plot_trajectory)
# add_subdirectory(trajectory_error)

# add_dependencies(${TRAJECTORY_ERROR} ${UTILS})
add_dependencies(${PLOT_TRAJECTORY} ${UTILS})
