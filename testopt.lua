        return {
          occupied_space_weight_0 = 1.,
          occupied_space_weight_1 = 16.,
	  
          intensity_cost_function_options_0 = {
            weight = 0.5,
            huber_scale = 55,
            intensity_threshold = 100,
          },
          translation_weight = 0.1,
          rotation_weight = 0.1,
          only_optimize_yaw = false,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 999,
            num_threads = 10,
          },
        }
