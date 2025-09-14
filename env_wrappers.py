from libero.libero.envs.env_wrapper import ControlEnv





class OnScreenRenderEnv(ControlEnv):
    """
    For visualization and evaluation.
    """

    def __init__(self, **kwargs):
        # kwargs["has_renderer"] = False
        # kwargs["has_offscreen_renderer"] = True
        kwargs["render_camera"] = "frontview"
        
        super().__init__(**kwargs)

    def _get_observations(self):
        return self.env._get_observations()