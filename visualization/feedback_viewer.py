"""Display measured hardware poses in MuJoCo without running a physics servo."""
import mujoco


def sync_feedback(backend, state):
    """Update a visualization-only backend, including all passive mimic joints.

    This deliberately sets qpos: it is a measured-pose display, not a simulated
    controller. No hardware calls are made here.
    """
    positions = backend.description.positions(state.positions_rad)
    for name, value in backend.description.expand(positions).items():
        joint_id = backend.model.joint(name).id
        backend.data.qpos[backend.model.jnt_qposadr[joint_id]] = value
        backend.data.qvel[backend.model.jnt_dofadr[joint_id]] = state.joint_velocities_rad_s[name]
    backend.target_rad = positions.copy()
    backend.reference_rad = positions.copy()
    backend.data.ctrl[:] = 0
    mujoco.mj_forward(backend.model, backend.data)


def open_viewer(stack, backend):
    import mujoco.viewer
    from visualization.record_video import camera_for
    viewer = stack.enter_context(mujoco.viewer.launch_passive(backend.model, backend.data))
    camera = camera_for(backend)
    viewer.cam.lookat[:] = camera.lookat
    viewer.cam.distance = camera.distance
    viewer.cam.azimuth = camera.azimuth
    viewer.cam.elevation = camera.elevation
    viewer.sync()
    return viewer
