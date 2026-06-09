from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from datetime import datetime

def generate_launch_description():
    """
    EKF2 (PX4) vs IEKF (KF-GINS) 并行对比测试 Launcher
    
    支持功能：
    - 同时运行 PX4 EKF2 和 KF-GINS IEKF
    - 自动时间同步
    - 实时误差计算
    - 可选的 rosbag 记录
    - 多场景支持
    """
    
    # ============ 参数声明 ============
    run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (required for SITL)'
    )

    kf_gins_param_file = DeclareLaunchArgument(
        'kf_gins_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('kf_gins_ros2_native'),
            'config',
            'kfgins_sim_fixed.yaml'
        ]),
        description='KF-GINS ROS2 param file (sim recommended)'
    )
    kf_gins_core_config_file = DeclareLaunchArgument(
        'kf_gins_core_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('kf_gins_ros2_native'),
            'config',
            'kf_gins_core_sim_no_sage.yaml'
        ]),
        description='KF-GINS core YAML loaded by the adapter (sim default disables Sage-Husa adaptive R)'
    )
    force_zero_antlever = DeclareLaunchArgument(
        'force_zero_antlever', default_value='true',
        description='Override adapter antenna lever arm to zero; keep true for historical baseline comparability'
    )
    use_gnss_llh_for_pose = DeclareLaunchArgument(
        'use_gnss_llh_for_pose', default_value='false',
        description='Use GNSS LLH directly for /kf_gins/odom pose (visualization)'
    )
    
    scenario = DeclareLaunchArgument(
        'scenario', default_value='open_field',
        description='Test scenario: open_field, urban_canyon, forest, indoor'
    )
    
    record_bag = DeclareLaunchArgument(
        'record_bag', default_value='true',
        description='Record rosbag for offline analysis'
    )
    
    start_rviz = DeclareLaunchArgument(
        'start_rviz', default_value='true',
        description='Start RViz for visualization'
    )
    
    enable_real_time_comparison = DeclareLaunchArgument(
        'enable_real_time_comparison', default_value='false',
        description='Enable real-time comparison node; disabled by default for flight stability'
    )

    publish_ekf2_state = DeclareLaunchArgument(
        'publish_ekf2_state', default_value='true',
        description='Publish EKF2 state topics for PlotJuggler'
    )
    publish_iekf_state = DeclareLaunchArgument(
        'publish_iekf_state', default_value='true',
        description='Publish IEKF state topics for PlotJuggler'
    )
    publish_aligned_iekf_state = DeclareLaunchArgument(
        'publish_aligned_iekf_state', default_value='true',
        description='Publish aligned IEKF state topics for PlotJuggler'
    )
    comparison_publish_named_metrics = DeclareLaunchArgument(
        'comparison_publish_named_metrics', default_value='true',
        description='Publish named comparison metric topics for live plotting'
    )
    comparison_publish_live_metrics = DeclareLaunchArgument(
        'comparison_publish_live_metrics', default_value='true',
        description='Publish legacy /comparison/metrics Float32MultiArray topic'
    )
    comparison_metrics_publish_rate = DeclareLaunchArgument(
        'comparison_metrics_publish_rate', default_value='50',
        description='Publish rate for real-time comparison metrics/state topics (Hz)'
    )
    comparison_metrics_log_period_sec = DeclareLaunchArgument(
        'comparison_metrics_log_period_sec', default_value='10.0',
        description='How often real_time_comparison prints metric summaries to its log'
    )
    iekf_raw_topic = DeclareLaunchArgument(
        'iekf_raw_topic', default_value='__disabled__',
        description='Optional raw IEKF odom topic for diagnostics; disabled by default for flight stability'
    )
    comparison_compute_raw_metrics = DeclareLaunchArgument(
        'comparison_compute_raw_metrics', default_value='false',
        description='Compute raw IEKF pairing and raw error metrics inside real_time_comparison; opt-in for raw diagnostics'
    )
    comparison_raw_callback_mode = DeclareLaunchArgument(
        'comparison_raw_callback_mode', default_value='store',
        description='Raw IEKF callback mode: store, count_only, or drop'
    )
    iekf_fallback_topic = DeclareLaunchArgument(
        'iekf_fallback_topic', default_value='/kf_gins/fallback_active',
        description='Bool topic that reports whether /kf_gins/odom is currently using GNSS fallback'
    )
    comparison_csv_path = DeclareLaunchArgument(
        'comparison_csv_path',
        default_value=os.path.join('/tmp', f'kf_gins_comparison_{run_tag}', 'comparison_metrics.csv'),
        description='Structured CSV output written by real_time_comparison.py'
    )
    gnss_update_debug_csv_path = DeclareLaunchArgument(
        'gnss_update_debug_csv_path', default_value='',
        description='Optional CSV output written by kf_gins_node for applied GNSS update diagnostics'
    )
    gnss_nis_debug_csv_path = DeclareLaunchArgument(
        'gnss_nis_debug_csv_path', default_value='',
        description='Optional low-rate CSV output written by kf_gins_node for GNSS S/NIS/acceptance diagnostics'
    )
    gnss_nis_debug_max_rate_hz = DeclareLaunchArgument(
        'gnss_nis_debug_max_rate_hz', default_value='2.0',
        description='Maximum GNSS NIS debug CSV write rate in Hz; 0 logs every GNSS position update'
    )
    state_update_debug_csv_path = DeclareLaunchArgument(
        'state_update_debug_csv_path', default_value='',
        description='Optional low-rate CSV output for state update dx/state/covariance diagnostics'
    )
    state_update_debug_max_rate_hz = DeclareLaunchArgument(
        'state_update_debug_max_rate_hz', default_value='2.0',
        description='Maximum heading-row write rate in the state update debug CSV; GNSS/reopen/hygiene rows are always logged, and 0 logs every event'
    )
    dtrq_runtime_feature_debug_csv_path = DeclareLaunchArgument(
        'dtrq_runtime_feature_debug_csv_path', default_value='',
        description='Optional default-off CSV output for DTRQ runtime feature diagnostics'
    )
    dtrq_runtime_feature_debug_max_rate_hz = DeclareLaunchArgument(
        'dtrq_runtime_feature_debug_max_rate_hz', default_value='2.0',
        description='Maximum DTRQ runtime feature CSV write rate in Hz; 0 logs every GNSS position update'
    )
    early_recovery_bias_feedback_debug_enable = DeclareLaunchArgument(
        'early_recovery_bias_feedback_debug_enable', default_value='false',
        description='Default-off log-only counterfactual for early GNSS-position vertical accbias feedback recovery gate'
    )
    early_recovery_bias_feedback_apply_enable = DeclareLaunchArgument(
        'early_recovery_bias_feedback_apply_enable', default_value='false',
        description='Default-off apply mode for early GNSS-position vertical accbias feedback recovery gate'
    )
    early_recovery_bias_feedback_history_sec = DeclareLaunchArgument(
        'early_recovery_bias_feedback_history_sec', default_value='10.0',
        description='Trailing history window in seconds for early recovery bias-feedback diagnostics'
    )
    early_recovery_bias_feedback_min_armed_time_sec = DeclareLaunchArgument(
        'early_recovery_bias_feedback_min_armed_time_sec', default_value='35.0',
        description='Minimum armed-time for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_max_armed_time_sec = DeclareLaunchArgument(
        'early_recovery_bias_feedback_max_armed_time_sec', default_value='95.0',
        description='Maximum armed-time for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_ba_z_mean_max_mps2 = DeclareLaunchArgument(
        'early_recovery_bias_feedback_ba_z_mean_max_mps2', default_value='-0.18',
        description='Trailing mean accbias-z threshold for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_residual_u_mean_max_m = DeclareLaunchArgument(
        'early_recovery_bias_feedback_residual_u_mean_max_m', default_value='-0.02',
        description='Trailing mean GNSS vertical residual max for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_core_gnss_u_mean_min_m = DeclareLaunchArgument(
        'early_recovery_bias_feedback_core_gnss_u_mean_min_m', default_value='0.02',
        description='Trailing mean core-GNSS vertical difference min for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_dx_ba_z_sum_max_mps2 = DeclareLaunchArgument(
        'early_recovery_bias_feedback_dx_ba_z_sum_max_mps2', default_value='0.0',
        description='Trailing cumulative dx_ba_z threshold for early recovery bias-feedback gate'
    )
    early_recovery_bias_feedback_min_history_rows = DeclareLaunchArgument(
        'early_recovery_bias_feedback_min_history_rows', default_value='5',
        description='Minimum GNSS-position state-update rows in the trailing early recovery history'
    )
    early_recovery_bias_feedback_negative_dx_scale = DeclareLaunchArgument(
        'early_recovery_bias_feedback_negative_dx_scale', default_value='0.0',
        description='Log-only selected negative dx_ba_z scale under active early recovery gate; 0 clips negative dx in the counterfactual'
    )
    horizontal_consistency_debug_csv_path = DeclareLaunchArgument(
        'horizontal_consistency_debug_csv_path', default_value='',
        description='Optional CSV output for default-off horizontal consistency supervisor diagnostics'
    )
    heading_update_debug_csv_path = DeclareLaunchArgument(
        'heading_update_debug_csv_path', default_value='',
        description='Optional CSV output written by kf_gins_node for every heading update/relock event'
    )
    state_publish_debug_csv_path = DeclareLaunchArgument(
        'state_publish_debug_csv_path', default_value='',
        description='Optional CSV output written by kf_gins_node for every state publish'
    )
    shadow_restore_publish_enable = DeclareLaunchArgument(
        'shadow_restore_publish_enable', default_value='false',
        description='Publish KFCore snapshots for restore-only shadow wiring'
    )
    shadow_restore_subscribe_enable = DeclareLaunchArgument(
        'shadow_restore_subscribe_enable', default_value='false',
        description='Subscribe to KFCore snapshots and restore this node core'
    )
    shadow_restore_topic = DeclareLaunchArgument(
        'shadow_restore_topic', default_value='/kf_gins/main_to_shadow_snapshot',
        description='Absolute topic used for main-to-shadow snapshot restore wiring'
    )
    shadow_restore_publish_after_core_sec = DeclareLaunchArgument(
        'shadow_restore_publish_after_core_sec', default_value='40.0',
        description='Core time threshold for the first snapshot publish'
    )
    shadow_restore_publish_once = DeclareLaunchArgument(
        'shadow_restore_publish_once', default_value='true',
        description='Publish only the first eligible snapshot'
    )
    shadow_restore_publish_period_sec = DeclareLaunchArgument(
        'shadow_restore_publish_period_sec', default_value='0.0',
        description='Optional repeated snapshot publish period when publish_once=false'
    )
    shadow_restore_max_age_sec = DeclareLaunchArgument(
        'shadow_restore_max_age_sec', default_value='0.75',
        description='Maximum accepted restore snapshot age in ROS time seconds; negative disables age gate'
    )
    shadow_restore_covariance_inflation_factor = DeclareLaunchArgument(
        'shadow_restore_covariance_inflation_factor', default_value='1.0',
        description='Covariance inflation factor applied by a restore subscriber'
    )
    shadow_restore_require_core_initialized = DeclareLaunchArgument(
        'shadow_restore_require_core_initialized', default_value='true',
        description='Require target core to have completed normal first-GNSS initialization before restore'
    )
    shadow_restore_clear_path_on_apply = DeclareLaunchArgument(
        'shadow_restore_clear_path_on_apply', default_value='true',
        description='Clear visualization path when applying a restore snapshot'
    )
    shadow_restore_event_csv_path = DeclareLaunchArgument(
        'shadow_restore_event_csv_path', default_value='',
        description='Optional CSV path for publish/apply shadow restore events'
    )
    shadow_supervisor_fsm_debug_enable = DeclareLaunchArgument(
        'shadow_supervisor_fsm_debug_enable', default_value='false',
        description='Enable read-only shadow supervisor FSM diagnostics; does not mux estimator output'
    )
    shadow_supervisor_fsm_reference_odom_topic = DeclareLaunchArgument(
        'shadow_supervisor_fsm_reference_odom_topic', default_value='/kf_gins/odom',
        description='Reference odometry topic used by read-only shadow supervisor diagnostics'
    )
    shadow_supervisor_fsm_debug_csv_path = DeclareLaunchArgument(
        'shadow_supervisor_fsm_debug_csv_path', default_value='',
        description='Optional CSV path for per-sample shadow supervisor FSM diagnostics'
    )
    shadow_supervisor_fsm_events_csv_path = DeclareLaunchArgument(
        'shadow_supervisor_fsm_events_csv_path', default_value='',
        description='Optional CSV path for shadow supervisor FSM state-transition events'
    )
    shadow_supervisor_fsm_debug_max_rate_hz = DeclareLaunchArgument(
        'shadow_supervisor_fsm_debug_max_rate_hz', default_value='10.0',
        description='Maximum rate for read-only shadow supervisor FSM debug rows; 0 disables rate limiting'
    )
    shadow_supervisor_fsm_reference_odom_max_age_sec = DeclareLaunchArgument(
        'shadow_supervisor_fsm_reference_odom_max_age_sec', default_value='0.5',
        description='Maximum accepted reference odometry age for shadow/main consistency diagnostics'
    )
    shadow_supervisor_fsm_gamma_guard_enter = DeclareLaunchArgument(
        'shadow_supervisor_fsm_gamma_guard_enter', default_value='1.05',
        description='Observation gamma threshold that blocks shadow readiness in the diagnostic FSM'
    )
    shadow_supervisor_fsm_gamma_shadow_max = DeclareLaunchArgument(
        'shadow_supervisor_fsm_gamma_shadow_max', default_value='1.05',
        description='Maximum gamma allowed for shadow process-candidate diagnostics'
    )
    shadow_supervisor_fsm_process_lambda_enter = DeclareLaunchArgument(
        'shadow_supervisor_fsm_process_lambda_enter', default_value='1.012',
        description='Lambda threshold for process-candidate diagnostics'
    )
    shadow_supervisor_fsm_process_score_min = DeclareLaunchArgument(
        'shadow_supervisor_fsm_process_score_min', default_value='0.30',
        description='Process score threshold for process-candidate diagnostics'
    )
    shadow_supervisor_fsm_warmup_sec = DeclareLaunchArgument(
        'shadow_supervisor_fsm_warmup_sec', default_value='5.0',
        description='Post-restore warmup time before shadow readiness checks'
    )
    shadow_supervisor_fsm_ready_confirm_sec = DeclareLaunchArgument(
        'shadow_supervisor_fsm_ready_confirm_sec', default_value='2.0',
        description='Continuous consistency time required before SHADOW_READY diagnostics'
    )
    shadow_supervisor_fsm_xy_delta_ready_max_m = DeclareLaunchArgument(
        'shadow_supervisor_fsm_xy_delta_ready_max_m', default_value='0.50',
        description='Maximum horizontal main-shadow odometry delta for shadow-ready diagnostics'
    )
    shadow_supervisor_fsm_z_delta_ready_max_m = DeclareLaunchArgument(
        'shadow_supervisor_fsm_z_delta_ready_max_m', default_value='0.50',
        description='Maximum vertical main-shadow odometry delta for shadow-ready diagnostics'
    )
    shadow_supervisor_fsm_vel_delta_ready_max_mps = DeclareLaunchArgument(
        'shadow_supervisor_fsm_vel_delta_ready_max_mps', default_value='0.30',
        description='Maximum velocity main-shadow odometry delta for shadow-ready diagnostics'
    )
    shadow_supervisor_fsm_yaw_delta_ready_max_deg = DeclareLaunchArgument(
        'shadow_supervisor_fsm_yaw_delta_ready_max_deg', default_value='5.0',
        description='Maximum yaw main-shadow odometry delta for shadow-ready diagnostics'
    )
    shadow_supervisor_fsm_allow_mixed_trigger = DeclareLaunchArgument(
        'shadow_supervisor_fsm_allow_mixed_trigger', default_value='false',
        description='Allow process-candidate diagnostics when gamma suggests mixed observation/process disturbance'
    )
    shadow_supervisor_fsm_observation_score_guard_enable = DeclareLaunchArgument(
        'shadow_supervisor_fsm_observation_score_guard_enable', default_value='false',
        description='Optionally treat observation_score as a hard diagnostic guard'
    )
    shadow_supervisor_fsm_observation_score_guard_enter = DeclareLaunchArgument(
        'shadow_supervisor_fsm_observation_score_guard_enter', default_value='0.50',
        description='Observation score guard threshold when the optional score guard is enabled'
    )
    shadow_supervisor_perf_proxy_publish_enable = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_publish_enable', default_value='false',
        description='Publish read-only residual/NIS performance proxy samples for a shadow supervisor'
    )
    shadow_supervisor_perf_proxy_subscribe_enable = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_subscribe_enable', default_value='false',
        description='Subscribe to read-only residual/NIS performance proxy samples for FSM debug CSV fields'
    )
    shadow_supervisor_perf_proxy_topic = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_topic', default_value='/kf_gins/supervisor_performance_proxy',
        description='Topic for read-only shadow supervisor residual/NIS performance proxy samples'
    )
    shadow_supervisor_perf_proxy_max_age_sec = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_max_age_sec', default_value='0.75',
        description='Maximum age for performance proxy samples used in FSM debug diagnostics'
    )
    shadow_supervisor_perf_proxy_short_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_short_window_sec', default_value='5.0',
        description='Short rolling window for residual/NIS performance proxy debug gaps'
    )
    shadow_supervisor_perf_proxy_long_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_long_window_sec', default_value='10.0',
        description='Long rolling window for residual/NIS performance proxy debug gaps'
    )
    shadow_supervisor_perf_proxy_residual_gap_soft_max_m = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_residual_gap_soft_max_m', default_value='0.0',
        description='Residual-gap threshold for soft performance proxy debug pass'
    )
    shadow_supervisor_perf_proxy_nis_gap_permission_max = DeclareLaunchArgument(
        'shadow_supervisor_perf_proxy_nis_gap_permission_max', default_value='0.0',
        description='NIS-gap threshold for high-confidence performance proxy debug permission'
    )
    shadow_supervisor_predictive_score_publish_enable = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_publish_enable', default_value='false',
        description='Publish default-off read-only GNSS prequential predictive-score samples'
    )
    shadow_supervisor_predictive_score_subscribe_enable = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_subscribe_enable', default_value='false',
        description='Subscribe to read-only GNSS predictive-score samples for shadow CSV diagnostics'
    )
    shadow_supervisor_predictive_score_debug_enable = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_debug_enable', default_value='false',
        description='Write default-off rq_shadow_predictive_score_debug.csv diagnostics'
    )
    shadow_supervisor_predictive_score_topic = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_topic', default_value='/kf_gins/supervisor_predictive_score',
        description='Topic for read-only GNSS prequential predictive-score samples'
    )
    shadow_supervisor_predictive_score_debug_csv_path = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_debug_csv_path', default_value='',
        description='Optional CSV path for GPPS v1 predictive-score diagnostics'
    )
    shadow_supervisor_predictive_score_source_id = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_source_id', default_value='rq_shadow',
        description='Source label written into the GPPS v1 predictive-score CSV'
    )
    shadow_supervisor_predictive_score_max_age_sec = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_max_age_sec', default_value='0.75',
        description='Maximum age for main predictive-score samples'
    )
    shadow_supervisor_predictive_score_stamp_tolerance_sec = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_stamp_tolerance_sec', default_value='0.20',
        description='Maximum GNSS stamp mismatch accepted for paired predictive-score rows'
    )
    shadow_supervisor_predictive_score_short_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_short_window_sec', default_value='5.0',
        description='Short rolling GPPS v1 window in seconds'
    )
    shadow_supervisor_predictive_score_long_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_long_window_sec', default_value='10.0',
        description='Long rolling GPPS v1 window in seconds'
    )
    shadow_supervisor_predictive_score_eval_std_h_m = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_eval_std_h_m', default_value='0.10',
        description='Common evaluator horizontal GNSS standard deviation for GPPS v1'
    )
    shadow_supervisor_predictive_score_eval_std_u_m = DeclareLaunchArgument(
        'shadow_supervisor_predictive_score_eval_std_u_m', default_value='0.20',
        description='Common evaluator vertical GNSS standard deviation for GPPS v1 diagnostics'
    )
    shadow_supervisor_velocity_predictive_score_publish_enable = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_publish_enable', default_value='false',
        description='Publish default-off read-only GNSS velocity prequential predictive-score samples'
    )
    shadow_supervisor_velocity_predictive_score_subscribe_enable = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_subscribe_enable', default_value='false',
        description='Subscribe to read-only GNSS velocity predictive-score samples for shadow CSV diagnostics'
    )
    shadow_supervisor_velocity_predictive_score_debug_enable = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_debug_enable', default_value='false',
        description='Write default-off rq_shadow_velocity_predictive_score_debug.csv diagnostics'
    )
    shadow_supervisor_velocity_predictive_score_topic = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_topic',
        default_value='/kf_gins/supervisor_velocity_predictive_score',
        description='Topic for read-only GNSS velocity prequential predictive-score samples'
    )
    shadow_supervisor_velocity_predictive_score_debug_csv_path = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_debug_csv_path', default_value='',
        description='Optional CSV path for GVPS v1 predictive-score diagnostics'
    )
    shadow_supervisor_velocity_predictive_score_source_id = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_source_id', default_value='rq_shadow',
        description='Source label written into the GVPS v1 predictive-score CSV'
    )
    shadow_supervisor_velocity_predictive_score_max_age_sec = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_max_age_sec', default_value='0.75',
        description='Maximum age for main velocity predictive-score samples'
    )
    shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec', default_value='0.20',
        description='Maximum GNSS stamp mismatch accepted for paired GVPS rows'
    )
    shadow_supervisor_velocity_predictive_score_short_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_short_window_sec', default_value='5.0',
        description='Short rolling GVPS v1 window in seconds'
    )
    shadow_supervisor_velocity_predictive_score_long_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_long_window_sec', default_value='10.0',
        description='Long rolling GVPS v1 window in seconds'
    )
    shadow_supervisor_velocity_predictive_score_eval_std_h_mps = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_eval_std_h_mps', default_value='0.20',
        description='Common evaluator horizontal GNSS velocity standard deviation for GVPS v1'
    )
    shadow_supervisor_velocity_predictive_score_eval_std_u_mps = DeclareLaunchArgument(
        'shadow_supervisor_velocity_predictive_score_eval_std_u_mps', default_value='0.30',
        description='Common evaluator vertical GNSS velocity standard deviation for GVPS v1 diagnostics'
    )
    shadow_supervisor_kinematic_predictive_score_publish_enable = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_publish_enable', default_value='false',
        description='Publish default-off read-only GNSS kinematic prequential predictive-score samples'
    )
    shadow_supervisor_kinematic_predictive_score_subscribe_enable = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_subscribe_enable', default_value='false',
        description='Subscribe to read-only GKPS samples for shadow CSV diagnostics'
    )
    shadow_supervisor_kinematic_predictive_score_debug_enable = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_debug_enable', default_value='false',
        description='Write default-off rq_shadow_kinematic_predictive_score_debug.csv diagnostics'
    )
    shadow_supervisor_kinematic_predictive_score_topic = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_topic',
        default_value='/kf_gins/supervisor_kinematic_predictive_score',
        description='Topic for read-only GNSS kinematic prequential predictive-score samples'
    )
    shadow_supervisor_kinematic_predictive_score_debug_csv_path = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_debug_csv_path', default_value='',
        description='Optional CSV path for GKPS v1 predictive-score diagnostics'
    )
    shadow_supervisor_kinematic_predictive_score_source_id = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_source_id', default_value='rq_shadow',
        description='Source label written into the GKPS v1 predictive-score CSV'
    )
    shadow_supervisor_kinematic_predictive_score_max_age_sec = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_max_age_sec', default_value='0.75',
        description='Maximum age for main kinematic predictive-score samples'
    )
    shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec', default_value='0.20',
        description='Maximum GNSS stamp mismatch accepted for paired GKPS rows'
    )
    shadow_supervisor_kinematic_predictive_score_short_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_short_window_sec', default_value='5.0',
        description='Short rolling GKPS v1 window in seconds'
    )
    shadow_supervisor_kinematic_predictive_score_long_window_sec = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_long_window_sec', default_value='10.0',
        description='Long rolling GKPS v1 window in seconds'
    )
    shadow_supervisor_kinematic_predictive_score_eval_std_h_m = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_eval_std_h_m', default_value='0.10',
        description='Common evaluator horizontal GNSS position standard deviation for GKPS v1'
    )
    shadow_supervisor_kinematic_predictive_score_eval_std_u_m = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_eval_std_u_m', default_value='0.20',
        description='Common evaluator vertical GNSS position standard deviation for GKPS v1 diagnostics'
    )
    shadow_supervisor_kinematic_predictive_score_eval_std_h_mps = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_eval_std_h_mps', default_value='0.20',
        description='Common evaluator horizontal GNSS velocity standard deviation for GKPS v1'
    )
    shadow_supervisor_kinematic_predictive_score_eval_std_u_mps = DeclareLaunchArgument(
        'shadow_supervisor_kinematic_predictive_score_eval_std_u_mps', default_value='0.30',
        description='Common evaluator vertical GNSS velocity standard deviation for GKPS v1 diagnostics'
    )
    segment_timing_gate_debug_csv_path = DeclareLaunchArgument(
        'segment_timing_gate_debug_csv_path', default_value='',
        description='Optional CSV output for default-off segment-smoothed publish lag gate diagnostics'
    )
    segment_timing_gate_segment_sec = DeclareLaunchArgument(
        'segment_timing_gate_segment_sec', default_value='2.0',
        description='Segment length in seconds for segment timing gate diagnostics'
    )
    segment_timing_gate_lag_threshold_sec = DeclareLaunchArgument(
        'segment_timing_gate_lag_threshold_sec', default_value='0.025',
        description='Mean publish stamp lag threshold in seconds for segment timing gate diagnostics/action'
    )
    segment_timing_gate_gnss_source_age_max_sec = DeclareLaunchArgument(
        'segment_timing_gate_gnss_source_age_max_sec', default_value='-1.0',
        description='Optional mean GNSS-source-age max threshold in seconds for segment timing gate diagnostics/action; negative disables this guard'
    )
    segment_timing_gate_core_gnss_along_min_enable = DeclareLaunchArgument(
        'segment_timing_gate_core_gnss_along_min_enable', default_value='false',
        description='Default-off guard: require running segment min of core-minus-GNSS along core velocity to cross threshold'
    )
    segment_timing_gate_core_gnss_along_min_threshold_m = DeclareLaunchArgument(
        'segment_timing_gate_core_gnss_along_min_threshold_m', default_value='0.0',
        description='Threshold in meters for the running core-minus-GNSS along-velocity segment minimum'
    )
    segment_timing_gate_projection_alpha = DeclareLaunchArgument(
        'segment_timing_gate_projection_alpha', default_value='1.0',
        description='Blend alpha for segment timing gate projection: raw + alpha * (PX4-sphere - raw)'
    )
    segment_timing_gate_projection_enable = DeclareLaunchArgument(
        'segment_timing_gate_projection_enable', default_value='false',
        description='Default-off action: apply PX4-sphere output projection only while the segment timing gate is active'
    )
    segment_timing_gate_projection_apply_mission = DeclareLaunchArgument(
        'segment_timing_gate_projection_apply_mission', default_value='true',
        description='Allow segment timing gate output projection during AUTO.MISSION'
    )
    segment_timing_gate_projection_apply_rtl = DeclareLaunchArgument(
        'segment_timing_gate_projection_apply_rtl', default_value='false',
        description='Allow segment timing gate output projection during AUTO.RTL'
    )
    segment_timing_gate_projection_apply_other = DeclareLaunchArgument(
        'segment_timing_gate_projection_apply_other', default_value='false',
        description='Allow segment timing gate output projection outside AUTO.MISSION/AUTO.RTL'
    )
    publish_state_after_gnss_update = DeclareLaunchArgument(
        'publish_state_after_gnss_update', default_value='true',
        description='Publish /kf_gins/odom immediately after GNSS update events; set false to publish only after propagation updates'
    )
    publish_px4_sphere_projection = DeclareLaunchArgument(
        'publish_px4_sphere_projection', default_value='false',
        description='Scale published IEKF ENU x/y to the PX4 spherical local-position projection; raw odom stays unscaled'
    )
    accbias_z_history_projection_enable = DeclareLaunchArgument(
        'accbias_z_history_projection_enable', default_value='false',
        description='Default-off action: apply alpha-scaled PX4-sphere output projection when accbias-z deep-history basin is active'
    )
    accbias_z_history_projection_apply_mission = DeclareLaunchArgument(
        'accbias_z_history_projection_apply_mission', default_value='true',
        description='Allow accbias-z history output projection during AUTO.MISSION'
    )
    accbias_z_history_projection_apply_rtl = DeclareLaunchArgument(
        'accbias_z_history_projection_apply_rtl', default_value='false',
        description='Allow accbias-z history output projection during AUTO.RTL'
    )
    accbias_z_history_projection_apply_other = DeclareLaunchArgument(
        'accbias_z_history_projection_apply_other', default_value='false',
        description='Allow accbias-z history output projection outside AUTO.MISSION/AUTO.RTL'
    )
    accbias_z_history_projection_alpha = DeclareLaunchArgument(
        'accbias_z_history_projection_alpha', default_value='1.2',
        description='Blend alpha for accbias-z history projection: raw + alpha * (PX4-sphere - raw)'
    )
    accbias_z_history_projection_deep_threshold_mps2 = DeclareLaunchArgument(
        'accbias_z_history_projection_deep_threshold_mps2', default_value='-0.205',
        description='Accbias-z threshold counted as deep history for the output projection selector'
    )
    accbias_z_history_projection_frac_threshold = DeclareLaunchArgument(
        'accbias_z_history_projection_frac_threshold', default_value='0.4',
        description='Minimum deep-history fraction for the accbias-z history output projection selector'
    )
    accbias_z_history_projection_history_start_sec = DeclareLaunchArgument(
        'accbias_z_history_projection_history_start_sec', default_value='40.0',
        description='Armed-time start for accbias-z history accumulation'
    )
    publish_stamp_mode = DeclareLaunchArgument(
        'publish_stamp_mode', default_value='ros_now',
        description='Timestamp mode for /kf_gins/odom: ros_now or core_fixed_offset'
    )
    publish_core_stamp_max_future_sec = DeclareLaunchArgument(
        'publish_core_stamp_max_future_sec', default_value='0.08',
        description='Maximum allowed future offset when publish_stamp_mode=core_fixed_offset'
    )
    publish_core_stamp_max_past_sec = DeclareLaunchArgument(
        'publish_core_stamp_max_past_sec', default_value='0.25',
        description='Maximum allowed past offset when publish_stamp_mode=core_fixed_offset'
    )
    publish_core_stamp_offset_bias_sec = DeclareLaunchArgument(
        'publish_core_stamp_offset_bias_sec', default_value='0.0',
        description='Signed bias added to the core-to-ROS publish timestamp offset when publish_stamp_mode=core_fixed_offset'
    )
    raw_odom_decimation = DeclareLaunchArgument(
        'raw_odom_decimation', default_value='50',
        description='Publish every Nth /kf_gins/odom_raw sample from kf_gins_node'
    )
    path_publish_rate_hz = DeclareLaunchArgument(
        'path_publish_rate_hz', default_value='5.0',
        description='Timer publish rate for /kf_gins/path; set 0.0 to disable the timer'
    )
    pose_decimation = DeclareLaunchArgument(
        'pose_decimation', default_value='5',
        description='Append/publish every Nth pose sample into /kf_gins/path'
    )
    max_path_points = DeclareLaunchArgument(
        'max_path_points', default_value='20000',
        description='Maximum number of poses retained in /kf_gins/path'
    )
    core_processing_enable = DeclareLaunchArgument(
        'core_processing_enable', default_value='true',
        description='Enable kf_gins_node core IMU/GNSS processing; false keeps subscriptions active but skips filter updates'
    )
    core_imu_decimation = DeclareLaunchArgument(
        'core_imu_decimation', default_value='1',
        description='Process every Nth IMU sample in the KF-GINS core; skipped samples are accumulated before propagation'
    )
    core_max_imu_rate_hz = DeclareLaunchArgument(
        'core_max_imu_rate_hz', default_value='0.0',
        description='Maximum KF-GINS core IMU propagation rate; 0 disables rate limiting'
    )
    armed_cruise_native_gnss_vel_override_enable = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_override_enable', default_value='true',
        description='Tighten native GNSS velocity std during armed late-cruise motion context'
    )
    armed_cruise_gnss_pos_override_enable = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_override_enable', default_value='true',
        description='Tighten GNSS position std during armed flight when fresh speed data is available'
    )
    armed_cruise_gnss_pos_override_apply_mission = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_override_apply_mission', default_value='true',
        description='Allow armed-cruise GNSS position std override in AUTO.MISSION'
    )
    armed_cruise_gnss_pos_override_apply_rtl = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_override_apply_rtl', default_value='true',
        description='Allow armed-cruise GNSS position std override in AUTO.RTL'
    )
    armed_cruise_gnss_pos_override_apply_other = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_override_apply_other', default_value='true',
        description='Allow armed-cruise GNSS position std override outside AUTO.MISSION/AUTO.RTL'
    )
    armed_cruise_gnss_pos_std_h_m = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_std_h_m', default_value='0.06',
        description='Horizontal GNSS position std used when armed cruise position override is active'
    )
    armed_cruise_gnss_pos_std_u_m = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_std_u_m', default_value='0.08',
        description='Vertical GNSS position std used when armed cruise position override is active'
    )
    armed_cruise_gnss_pos_residual_boost_enable = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_residual_boost_enable', default_value='false',
        description='Temporarily tighten GNSS position std when the recent cruise GNSS position residual stays large'
    )
    armed_cruise_gnss_pos_residual_boost_threshold_m = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_residual_boost_threshold_m', default_value='0.12',
        description='Previous GNSS horizontal position residual threshold that arms the temporary cruise position-std boost'
    )
    armed_cruise_gnss_pos_residual_boost_hold_sec = DeclareLaunchArgument(
        'armed_cruise_gnss_pos_residual_boost_hold_sec', default_value='20.0',
        description='How long to keep the temporary cruise position-std boost active after a large residual hit'
    )
    gnss_position_response_boost_enable = DeclareLaunchArgument(
        'gnss_position_response_boost_enable', default_value='false',
        description='Enable default-off GNSS position response boost by tightening horizontal position std on large residuals'
    )
    gnss_position_response_boost_apply_mission = DeclareLaunchArgument(
        'gnss_position_response_boost_apply_mission', default_value='true',
        description='Allow GNSS position response boost during AUTO.MISSION'
    )
    gnss_position_response_boost_apply_rtl = DeclareLaunchArgument(
        'gnss_position_response_boost_apply_rtl', default_value='false',
        description='Allow GNSS position response boost during AUTO.RTL'
    )
    gnss_position_response_boost_residual_start_h_m = DeclareLaunchArgument(
        'gnss_position_response_boost_residual_start_h_m', default_value='0.25',
        description='Horizontal GNSS/core residual where position response boost starts'
    )
    gnss_position_response_boost_residual_full_h_m = DeclareLaunchArgument(
        'gnss_position_response_boost_residual_full_h_m', default_value='0.45',
        description='Horizontal GNSS/core residual where position response boost reaches full tightening'
    )
    gnss_position_response_boost_min_horizontal_speed_mps = DeclareLaunchArgument(
        'gnss_position_response_boost_min_horizontal_speed_mps', default_value='0.5',
        description='Minimum horizontal speed for GNSS position response boost'
    )
    gnss_position_response_boost_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'gnss_position_response_boost_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for GNSS position response boost'
    )
    gnss_position_response_boost_persistence_updates = DeclareLaunchArgument(
        'gnss_position_response_boost_persistence_updates', default_value='1',
        description='Consecutive GNSS updates required before position response boost applies'
    )
    gnss_position_response_boost_std_min_h_m = DeclareLaunchArgument(
        'gnss_position_response_boost_std_min_h_m', default_value='0.04',
        description='Minimum horizontal GNSS position std while response boost is fully active'
    )
    gnss_position_response_boost_std_max_h_m = DeclareLaunchArgument(
        'gnss_position_response_boost_std_max_h_m', default_value='0.06',
        description='Maximum horizontal GNSS position std target while response boost is active'
    )
    gnss_position_gain_response_enable = DeclareLaunchArgument(
        'gnss_position_gain_response_enable', default_value='false',
        description='Enable default-off GNSS position gain response when residual/NIS are high and prior position gain was low'
    )
    gnss_position_gain_response_apply_mission = DeclareLaunchArgument(
        'gnss_position_gain_response_apply_mission', default_value='true',
        description='Allow GNSS position gain response during AUTO.MISSION'
    )
    gnss_position_gain_response_apply_rtl = DeclareLaunchArgument(
        'gnss_position_gain_response_apply_rtl', default_value='false',
        description='Allow GNSS position gain response during AUTO.RTL'
    )
    gnss_position_gain_response_require_armed_cruise = DeclareLaunchArgument(
        'gnss_position_gain_response_require_armed_cruise', default_value='true',
        description='Require armed cruise context for GNSS position gain response'
    )
    gnss_position_gain_response_block_turning = DeclareLaunchArgument(
        'gnss_position_gain_response_block_turning', default_value='true',
        description='Block GNSS position gain response during detected turns'
    )
    gnss_position_gain_response_block_post_turn = DeclareLaunchArgument(
        'gnss_position_gain_response_block_post_turn', default_value='true',
        description='Block GNSS position gain response in post-turn context'
    )
    gnss_position_gain_response_residual_start_h_m = DeclareLaunchArgument(
        'gnss_position_gain_response_residual_start_h_m', default_value='0.15',
        description='Horizontal GNSS/core residual where position gain response starts'
    )
    gnss_position_gain_response_residual_full_h_m = DeclareLaunchArgument(
        'gnss_position_gain_response_residual_full_h_m', default_value='0.35',
        description='Horizontal GNSS/core residual where position gain response reaches full tightening'
    )
    gnss_position_gain_response_hnis_start = DeclareLaunchArgument(
        'gnss_position_gain_response_hnis_start', default_value='6.0',
        description='Approximate horizontal NIS where position gain response starts'
    )
    gnss_position_gain_response_hnis_full = DeclareLaunchArgument(
        'gnss_position_gain_response_hnis_full', default_value='20.0',
        description='Approximate horizontal NIS where position gain response reaches full tightening'
    )
    gnss_position_gain_response_prev_gain_low = DeclareLaunchArgument(
        'gnss_position_gain_response_prev_gain_low', default_value='0.10',
        description='Previous dx/residual ratio treated as fully low position response'
    )
    gnss_position_gain_response_prev_gain_high = DeclareLaunchArgument(
        'gnss_position_gain_response_prev_gain_high', default_value='0.18',
        description='Previous dx/residual ratio where position gain response fades to zero'
    )
    gnss_position_gain_response_min_horizontal_speed_mps = DeclareLaunchArgument(
        'gnss_position_gain_response_min_horizontal_speed_mps', default_value='3.0',
        description='Minimum horizontal speed for GNSS position gain response'
    )
    gnss_position_gain_response_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'gnss_position_gain_response_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for GNSS position gain response'
    )
    gnss_position_gain_response_persistence_updates = DeclareLaunchArgument(
        'gnss_position_gain_response_persistence_updates', default_value='1',
        description='Consecutive GNSS updates required before position gain response applies'
    )
    gnss_position_gain_response_std_min_h_m = DeclareLaunchArgument(
        'gnss_position_gain_response_std_min_h_m', default_value='0.03',
        description='Minimum horizontal GNSS position std while gain response is fully active'
    )
    gnss_position_gain_response_std_max_h_m = DeclareLaunchArgument(
        'gnss_position_gain_response_std_max_h_m', default_value='0.06',
        description='Maximum horizontal GNSS position std target while gain response is active'
    )
    gnss_velocity_outward_damping_enable = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_enable', default_value='false',
        description='Enable default-off GNSS velocity outward damping based on core-vs-GNSS radial velocity residual'
    )
    gnss_velocity_outward_damping_apply_mission = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_apply_mission', default_value='true',
        description='Allow GNSS velocity outward damping during AUTO.MISSION'
    )
    gnss_velocity_outward_damping_apply_rtl = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_apply_rtl', default_value='false',
        description='Allow GNSS velocity outward damping during AUTO.RTL'
    )
    gnss_velocity_outward_damping_min_core_gnss_diff_h_m = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_min_core_gnss_diff_h_m', default_value='0.20',
        description='Minimum horizontal core-vs-GNSS position difference before outward damping can trigger'
    )
    gnss_velocity_outward_damping_radial_start_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_radial_start_mps', default_value='0.03',
        description='Radial outward core-vs-native velocity residual where outward damping starts'
    )
    gnss_velocity_outward_damping_radial_full_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_radial_full_mps', default_value='0.12',
        description='Radial outward core-vs-native velocity residual where outward damping reaches full tightening'
    )
    gnss_velocity_outward_damping_min_horizontal_speed_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_min_horizontal_speed_mps', default_value='0.5',
        description='Minimum horizontal speed for GNSS velocity outward damping'
    )
    gnss_velocity_outward_damping_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for GNSS velocity outward damping'
    )
    gnss_velocity_outward_damping_persistence_updates = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_persistence_updates', default_value='1',
        description='Consecutive GNSS updates required before outward damping applies'
    )
    gnss_velocity_outward_damping_hold_updates = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_hold_updates', default_value='0',
        description='Hold active outward damping for this many subsequent positive-below-trigger updates'
    )
    gnss_velocity_outward_damping_std_min_h_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_std_min_h_mps', default_value='0.02',
        description='Minimum horizontal GNSS velocity std while outward damping is fully active'
    )
    gnss_velocity_outward_damping_std_max_h_mps = DeclareLaunchArgument(
        'gnss_velocity_outward_damping_std_max_h_mps', default_value='0.05',
        description='Maximum horizontal GNSS velocity std target while outward damping is active'
    )
    turn_postturn_native_velocity_deweight_enable = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_enable', default_value='false',
        description='Enable default-off turn/post-turn native GNSS velocity deweighting on velocity coupling residuals'
    )
    turn_postturn_native_velocity_deweight_apply_mission = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_apply_mission', default_value='true',
        description='Allow turn/post-turn native velocity deweighting during AUTO.MISSION'
    )
    turn_postturn_native_velocity_deweight_apply_rtl = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_apply_rtl', default_value='false',
        description='Allow turn/post-turn native velocity deweighting during AUTO.RTL'
    )
    turn_postturn_native_velocity_deweight_apply_turning = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_apply_turning', default_value='true',
        description='Allow native velocity deweighting during active turning context'
    )
    turn_postturn_native_velocity_deweight_apply_post_turn = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_apply_post_turn', default_value='true',
        description='Allow native velocity deweighting during post-turn context'
    )
    turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m', default_value='0.15',
        description='Minimum horizontal core-vs-GNSS position difference before native velocity deweighting can trigger'
    )
    turn_postturn_native_velocity_deweight_radial_abs_start_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_radial_abs_start_mps', default_value='0.08',
        description='Absolute radial core-vs-native velocity residual where deweighting starts'
    )
    turn_postturn_native_velocity_deweight_radial_abs_full_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_radial_abs_full_mps', default_value='0.28',
        description='Absolute radial core-vs-native velocity residual where deweighting reaches full strength'
    )
    turn_postturn_native_velocity_deweight_core_residual_start_h_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_core_residual_start_h_mps', default_value='0.08',
        description='Horizontal core-vs-native velocity residual where deweighting starts'
    )
    turn_postturn_native_velocity_deweight_core_residual_full_h_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_core_residual_full_h_mps', default_value='0.30',
        description='Horizontal core-vs-native velocity residual where deweighting reaches full strength'
    )
    turn_postturn_native_velocity_deweight_min_horizontal_speed_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_min_horizontal_speed_mps', default_value='0.5',
        description='Minimum horizontal speed for turn/post-turn native velocity deweighting'
    )
    turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for turn/post-turn native velocity deweighting'
    )
    turn_postturn_native_velocity_deweight_persistence_updates = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_persistence_updates', default_value='1',
        description='Consecutive GNSS updates required before native velocity deweighting applies'
    )
    turn_postturn_native_velocity_deweight_std_min_h_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_std_min_h_mps', default_value='0.10',
        description='Minimum horizontal native GNSS velocity std target while deweighting is active'
    )
    turn_postturn_native_velocity_deweight_std_max_h_mps = DeclareLaunchArgument(
        'turn_postturn_native_velocity_deweight_std_max_h_mps', default_value='0.18',
        description='Maximum horizontal native GNSS velocity std target while deweighting is active'
    )
    phase_error_memory_debug_enable = DeclareLaunchArgument(
        'phase_error_memory_debug_enable', default_value='false',
        description='Enable read-only phase-error memory diagnostic flags in GNSS debug CSVs'
    )
    phase_error_memory_debug_residual_threshold_h_m = DeclareLaunchArgument(
        'phase_error_memory_debug_residual_threshold_h_m', default_value='0.16',
        description='Horizontal GNSS residual threshold for the phase-error memory pressure label'
    )
    phase_error_memory_debug_dx_over_residual_threshold = DeclareLaunchArgument(
        'phase_error_memory_debug_dx_over_residual_threshold', default_value='0.22',
        description='Maximum dx/residual ratio for the phase-error memory weak-update label'
    )
    phase_error_memory_debug_recent_turnpost_hold_sec = DeclareLaunchArgument(
        'phase_error_memory_debug_recent_turnpost_hold_sec', default_value='15.0',
        description='Seconds after turn/post-turn context treated as recent phase memory'
    )
    adaptive_gnss_pos_weight_enable = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_enable', default_value='false',
        description='Enable default-off adaptive GNSS horizontal position observation weighting'
    )
    adaptive_gnss_pos_weight_apply_mission = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_apply_mission', default_value='true',
        description='Allow adaptive GNSS position weighting during AUTO.MISSION'
    )
    adaptive_gnss_pos_weight_apply_rtl = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_apply_rtl', default_value='false',
        description='Allow adaptive GNSS position weighting during AUTO.RTL'
    )
    adaptive_gnss_pos_weight_trigger_source = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_trigger_source', default_value='residual_h',
        description='Adaptive GNSS position weighting trigger source: residual_h or nis_h'
    )
    adaptive_gnss_pos_weight_floor_min_h_m = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_floor_min_h_m', default_value='0.08',
        description='Minimum adaptive horizontal GNSS position std floor'
    )
    adaptive_gnss_pos_weight_floor_nominal_h_m = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_floor_nominal_h_m', default_value='0.12',
        description='Nominal adaptive horizontal GNSS position std floor'
    )
    adaptive_gnss_pos_weight_floor_max_h_m = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_floor_max_h_m', default_value='0.16',
        description='Maximum adaptive horizontal GNSS position std floor'
    )
    adaptive_gnss_pos_weight_residual_start_h_m = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_residual_start_h_m', default_value='0.24',
        description='Horizontal GNSS residual threshold where adaptive weighting starts'
    )
    adaptive_gnss_pos_weight_residual_full_h_m = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_residual_full_h_m', default_value='0.45',
        description='Horizontal GNSS residual threshold where adaptive weighting reaches max floor'
    )
    adaptive_gnss_pos_weight_nis_start_h_2d = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_nis_start_h_2d', default_value='5.991',
        description='Horizontal 2D GNSS NIS threshold where adaptive weighting starts'
    )
    adaptive_gnss_pos_weight_nis_full_h_2d = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_nis_full_h_2d', default_value='20.0',
        description='Horizontal 2D GNSS NIS threshold where adaptive weighting reaches max floor'
    )
    adaptive_gnss_pos_weight_nis_max_age_sec = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_nis_max_age_sec', default_value='2.0',
        description='Maximum age for previous GNSS position hNIS when nis_h trigger is used'
    )
    adaptive_gnss_pos_weight_persistence_updates = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_persistence_updates', default_value='5',
        description='Consecutive GNSS updates required before adaptive weighting activates'
    )
    adaptive_gnss_pos_weight_attack_sec = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_attack_sec', default_value='0.8',
        description='Adaptive GNSS position std floor attack time constant'
    )
    adaptive_gnss_pos_weight_decay_sec = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_decay_sec', default_value='1.2',
        description='Adaptive GNSS position std floor decay time constant'
    )
    adaptive_gnss_pos_weight_armed_cruise_gain = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_armed_cruise_gain', default_value='1.0',
        description='Adaptive weighting score gain during armed-cruise context'
    )
    adaptive_gnss_pos_weight_turn_gain = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_turn_gain', default_value='1.15',
        description='Adaptive weighting score gain during turn context'
    )
    adaptive_gnss_pos_weight_post_turn_gain = DeclareLaunchArgument(
        'adaptive_gnss_pos_weight_post_turn_gain', default_value='1.15',
        description='Adaptive weighting score gain during post-turn context'
    )
    horizontal_consistency_supervisor_enable = DeclareLaunchArgument(
        'horizontal_consistency_supervisor_enable', default_value='false',
        description='Enable read-only horizontal consistency supervisor scoring'
    )
    horizontal_consistency_apply_mission = DeclareLaunchArgument(
        'horizontal_consistency_apply_mission', default_value='true',
        description='Allow horizontal consistency supervisor context during AUTO.MISSION'
    )
    horizontal_consistency_apply_rtl = DeclareLaunchArgument(
        'horizontal_consistency_apply_rtl', default_value='false',
        description='Allow horizontal consistency supervisor context during AUTO.RTL'
    )
    horizontal_consistency_nis_start_h_2d = DeclareLaunchArgument(
        'horizontal_consistency_nis_start_h_2d', default_value='5.991',
        description='Horizontal 2D GNSS NIS threshold where consistency score starts'
    )
    horizontal_consistency_nis_full_h_2d = DeclareLaunchArgument(
        'horizontal_consistency_nis_full_h_2d', default_value='20.0',
        description='Horizontal 2D GNSS NIS threshold where consistency score reaches full'
    )
    horizontal_consistency_residual_start_h_m = DeclareLaunchArgument(
        'horizontal_consistency_residual_start_h_m', default_value='0.20',
        description='Horizontal GNSS residual threshold where consistency score starts'
    )
    horizontal_consistency_residual_full_h_m = DeclareLaunchArgument(
        'horizontal_consistency_residual_full_h_m', default_value='0.45',
        description='Horizontal GNSS residual threshold where consistency score reaches full'
    )
    horizontal_consistency_core_gnss_start_h_m = DeclareLaunchArgument(
        'horizontal_consistency_core_gnss_start_h_m', default_value='0.20',
        description='Core-vs-GNSS horizontal difference threshold where consistency score starts'
    )
    horizontal_consistency_core_gnss_full_h_m = DeclareLaunchArgument(
        'horizontal_consistency_core_gnss_full_h_m', default_value='0.45',
        description='Core-vs-GNSS horizontal difference threshold where consistency score reaches full'
    )
    horizontal_consistency_heading_residual_start_deg = DeclareLaunchArgument(
        'horizontal_consistency_heading_residual_start_deg', default_value='0.8',
        description='Latest heading residual threshold where consistency score starts'
    )
    horizontal_consistency_heading_residual_full_deg = DeclareLaunchArgument(
        'horizontal_consistency_heading_residual_full_deg', default_value='2.0',
        description='Latest heading residual threshold where consistency score reaches full'
    )
    horizontal_consistency_score_trigger = DeclareLaunchArgument(
        'horizontal_consistency_score_trigger', default_value='0.75',
        description='Consistency score threshold that starts persistence counting'
    )
    horizontal_consistency_persistence_updates = DeclareLaunchArgument(
        'horizontal_consistency_persistence_updates', default_value='3',
        description='Consecutive accepted GNSS updates required before proposed action becomes active'
    )
    horizontal_consistency_min_horizontal_speed_mps = DeclareLaunchArgument(
        'horizontal_consistency_min_horizontal_speed_mps', default_value='3.0',
        description='Minimum horizontal speed for consistency supervisor motion gate'
    )
    horizontal_consistency_max_vertical_speed_mps = DeclareLaunchArgument(
        'horizontal_consistency_max_vertical_speed_mps', default_value='0.8',
        description='Maximum absolute vertical speed for consistency supervisor motion gate'
    )
    mission_cov_hygiene_enable = DeclareLaunchArgument(
        'mission_cov_hygiene_enable', default_value='false',
        description='Enable default-off mission horizontal position covariance hygiene'
    )
    mission_cov_hygiene_apply_mission = DeclareLaunchArgument(
        'mission_cov_hygiene_apply_mission', default_value='true',
        description='Allow covariance hygiene during AUTO.MISSION'
    )
    mission_cov_hygiene_apply_rtl = DeclareLaunchArgument(
        'mission_cov_hygiene_apply_rtl', default_value='false',
        description='Allow covariance hygiene during AUTO.RTL'
    )
    mission_cov_hygiene_hnis_start = DeclareLaunchArgument(
        'mission_cov_hygiene_hnis_start', default_value='6.0',
        description='Horizontal GNSS NIS threshold where covariance hygiene score starts'
    )
    mission_cov_hygiene_hnis_full = DeclareLaunchArgument(
        'mission_cov_hygiene_hnis_full', default_value='20.0',
        description='Horizontal GNSS NIS threshold where covariance hygiene score reaches full'
    )
    mission_cov_hygiene_resid_start_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_resid_start_h_m', default_value='0.15',
        description='Horizontal GNSS residual threshold where covariance hygiene score starts'
    )
    mission_cov_hygiene_resid_full_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_resid_full_h_m', default_value='0.35',
        description='Horizontal GNSS residual threshold where covariance hygiene score reaches full'
    )
    mission_cov_hygiene_pos_std_tight_lo_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_pos_std_tight_lo_h_m', default_value='0.025',
        description='Horizontal position std treated as fully over-tight for covariance hygiene'
    )
    mission_cov_hygiene_pos_std_tight_hi_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_pos_std_tight_hi_h_m', default_value='0.050',
        description='Horizontal position std where over-tight covariance score starts'
    )
    mission_cov_hygiene_dx_ratio_low = DeclareLaunchArgument(
        'mission_cov_hygiene_dx_ratio_low', default_value='0.15',
        description='Prior GNSS update dx/residual ratio treated as fully weak'
    )
    mission_cov_hygiene_dx_ratio_high = DeclareLaunchArgument(
        'mission_cov_hygiene_dx_ratio_high', default_value='0.35',
        description='Prior GNSS update dx/residual ratio where weak-update score starts'
    )
    mission_cov_hygiene_persistence_updates = DeclareLaunchArgument(
        'mission_cov_hygiene_persistence_updates', default_value='3',
        description='Consecutive accepted GNSS updates required before covariance hygiene activates'
    )
    mission_cov_hygiene_attack_sec = DeclareLaunchArgument(
        'mission_cov_hygiene_attack_sec', default_value='4.0',
        description='Covariance hygiene floor attack time constant'
    )
    mission_cov_hygiene_decay_sec = DeclareLaunchArgument(
        'mission_cov_hygiene_decay_sec', default_value='15.0',
        description='Covariance hygiene floor decay time constant'
    )
    mission_cov_hygiene_floor_min_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_floor_min_h_m', default_value='0.06',
        description='Minimum horizontal position covariance hygiene std floor'
    )
    mission_cov_hygiene_floor_nominal_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_floor_nominal_h_m', default_value='0.08',
        description='Nominal horizontal position covariance hygiene std floor'
    )
    mission_cov_hygiene_floor_max_h_m = DeclareLaunchArgument(
        'mission_cov_hygiene_floor_max_h_m', default_value='0.12',
        description='Maximum horizontal position covariance hygiene std floor'
    )
    mission_cov_hygiene_offdiag_corr_limit = DeclareLaunchArgument(
        'mission_cov_hygiene_offdiag_corr_limit', default_value='0.70',
        description='Maximum horizontal position off-diagonal correlation after covariance hygiene'
    )
    turn_rate_propagation_noise_probe_enable = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_enable', default_value='false',
        description='Enable default-off TPN1a turn-rate propagation Q scaling probe'
    )
    turn_rate_propagation_noise_probe_apply_mission = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_apply_mission', default_value='true',
        description='Allow TPN1a propagation Q scaling during AUTO.MISSION'
    )
    turn_rate_propagation_noise_probe_apply_rtl = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_apply_rtl', default_value='false',
        description='Allow TPN1a propagation Q scaling during AUTO.RTL'
    )
    turn_rate_propagation_noise_probe_gyro_start_deg_s = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_gyro_start_deg_s', default_value='6.0',
        description='Gyro norm where TPN1a turn-rate score starts'
    )
    turn_rate_propagation_noise_probe_gyro_full_deg_s = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_gyro_full_deg_s', default_value='20.0',
        description='Gyro norm where TPN1a turn-rate score reaches full'
    )
    turn_rate_propagation_noise_probe_min_horizontal_speed_mps = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_min_horizontal_speed_mps', default_value='0.5',
        description='Minimum horizontal speed for TPN1a motion gate'
    )
    turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for TPN1a motion gate'
    )
    turn_rate_propagation_noise_probe_arw_q_scale_max = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_arw_q_scale_max', default_value='9.0',
        description='Maximum continuous-time gyro ARW variance multiplier for TPN1a'
    )
    turn_rate_propagation_noise_probe_vrw_q_scale_max = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_vrw_q_scale_max', default_value='4.0',
        description='Maximum continuous-time accel VRW variance multiplier for TPN1a'
    )
    turn_rate_propagation_noise_probe_gyrbias_q_scale_max = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_gyrbias_q_scale_max', default_value='4.0',
        description='Maximum gyro-bias random-walk variance multiplier for TPN1a'
    )
    turn_rate_propagation_noise_probe_accbias_q_scale_max = DeclareLaunchArgument(
        'turn_rate_propagation_noise_probe_accbias_q_scale_max', default_value='2.0',
        description='Maximum accel-bias random-walk variance multiplier for TPN1a'
    )
    accbias_z_propagation_probe_enable = DeclareLaunchArgument(
        'accbias_z_propagation_probe_enable', default_value='false',
        description='Enable default-off vertical accelerometer-bias propagation probe'
    )
    accbias_z_propagation_probe_apply_noise_scale = DeclareLaunchArgument(
        'accbias_z_propagation_probe_apply_noise_scale', default_value='false',
        description='Apply propagation Q scaling for the accbias-z probe; false logs diagnostic-only'
    )
    accbias_z_propagation_probe_apply_mission = DeclareLaunchArgument(
        'accbias_z_propagation_probe_apply_mission', default_value='true',
        description='Allow accbias-z propagation probe during AUTO.MISSION'
    )
    accbias_z_propagation_probe_apply_rtl = DeclareLaunchArgument(
        'accbias_z_propagation_probe_apply_rtl', default_value='false',
        description='Allow accbias-z propagation probe during AUTO.RTL'
    )
    accbias_z_propagation_probe_trigger_mps2 = DeclareLaunchArgument(
        'accbias_z_propagation_probe_trigger_mps2', default_value='-0.18',
        description='Negative vertical accelerometer bias where accbias-z probe starts'
    )
    accbias_z_propagation_probe_full_mps2 = DeclareLaunchArgument(
        'accbias_z_propagation_probe_full_mps2', default_value='-0.24',
        description='Negative vertical accelerometer bias where accbias-z probe reaches full score'
    )
    accbias_z_propagation_probe_min_horizontal_speed_mps = DeclareLaunchArgument(
        'accbias_z_propagation_probe_min_horizontal_speed_mps', default_value='3.0',
        description='Minimum horizontal speed for accbias-z probe motion gate'
    )
    accbias_z_propagation_probe_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'accbias_z_propagation_probe_max_abs_vertical_speed_mps', default_value='1.0',
        description='Maximum absolute vertical speed for accbias-z probe motion gate'
    )
    accbias_z_propagation_probe_arw_q_scale_max = DeclareLaunchArgument(
        'accbias_z_propagation_probe_arw_q_scale_max', default_value='1.0',
        description='Maximum continuous-time gyro ARW variance multiplier for accbias-z probe'
    )
    accbias_z_propagation_probe_vrw_q_scale_max = DeclareLaunchArgument(
        'accbias_z_propagation_probe_vrw_q_scale_max', default_value='3.0',
        description='Maximum continuous-time accel VRW variance multiplier for accbias-z probe'
    )
    accbias_z_propagation_probe_gyrbias_q_scale_max = DeclareLaunchArgument(
        'accbias_z_propagation_probe_gyrbias_q_scale_max', default_value='1.0',
        description='Maximum gyro-bias random-walk variance multiplier for accbias-z probe'
    )
    accbias_z_propagation_probe_accbias_q_scale_max = DeclareLaunchArgument(
        'accbias_z_propagation_probe_accbias_q_scale_max', default_value='6.0',
        description='Maximum accel-bias random-walk variance multiplier for accbias-z probe'
    )
    motion_gnss_pos_weight_enable = DeclareLaunchArgument(
        'motion_gnss_pos_weight_enable', default_value='false',
        description='Enable default-off motion/phase scheduled GNSS horizontal position weighting'
    )
    motion_gnss_pos_weight_apply_mission = DeclareLaunchArgument(
        'motion_gnss_pos_weight_apply_mission', default_value='true',
        description='Allow motion scheduled GNSS position weighting during AUTO.MISSION'
    )
    motion_gnss_pos_weight_apply_rtl = DeclareLaunchArgument(
        'motion_gnss_pos_weight_apply_rtl', default_value='false',
        description='Allow motion scheduled GNSS position weighting during AUTO.RTL'
    )
    motion_gnss_pos_weight_hspeed_start_mps = DeclareLaunchArgument(
        'motion_gnss_pos_weight_hspeed_start_mps', default_value='3.5',
        description='Horizontal speed where motion scheduled GNSS position std starts increasing'
    )
    motion_gnss_pos_weight_hspeed_full_mps = DeclareLaunchArgument(
        'motion_gnss_pos_weight_hspeed_full_mps', default_value='5.0',
        description='Horizontal speed where motion scheduled GNSS position std reaches max'
    )
    motion_gnss_pos_weight_max_abs_vspeed_mps = DeclareLaunchArgument(
        'motion_gnss_pos_weight_max_abs_vspeed_mps', default_value='0.35',
        description='Maximum absolute vertical speed for motion scheduled GNSS position weighting'
    )
    motion_gnss_pos_weight_std_min_h_m = DeclareLaunchArgument(
        'motion_gnss_pos_weight_std_min_h_m', default_value='0.08',
        description='Minimum scheduled horizontal GNSS position std'
    )
    motion_gnss_pos_weight_std_max_h_m = DeclareLaunchArgument(
        'motion_gnss_pos_weight_std_max_h_m', default_value='0.14',
        description='Maximum scheduled horizontal GNSS position std'
    )
    gnss_pos_recovery_weight_enable = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_enable', default_value='false',
        description='Enable default-off bounded GNSS position recovery weighting'
    )
    gnss_pos_recovery_weight_apply_mission = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_apply_mission', default_value='true',
        description='Allow GNSS position recovery weighting during AUTO.MISSION'
    )
    gnss_pos_recovery_weight_apply_rtl = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_apply_rtl', default_value='false',
        description='Allow GNSS position recovery weighting during AUTO.RTL'
    )
    gnss_pos_recovery_weight_residual_start_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_residual_start_h_m', default_value='0.32',
        description='Horizontal position residual threshold where recovery triggering starts'
    )
    gnss_pos_recovery_weight_residual_full_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_residual_full_h_m', default_value='0.45',
        description='Horizontal position residual threshold where recovery residual score is full'
    )
    gnss_pos_recovery_weight_core_start_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_core_start_h_m', default_value='0.30',
        description='Core-vs-GNSS horizontal difference where weak recovery modifier starts'
    )
    gnss_pos_recovery_weight_core_full_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_core_full_h_m', default_value='0.55',
        description='Core-vs-GNSS horizontal difference where weak recovery modifier is full'
    )
    gnss_pos_recovery_weight_core_score_gain = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_core_score_gain', default_value='0.25',
        description='Weak score gain applied to core-vs-GNSS divergence in recovery weighting'
    )
    gnss_pos_recovery_weight_min_horizontal_speed_mps = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_min_horizontal_speed_mps', default_value='3.0',
        description='Minimum horizontal speed for GNSS position recovery motion gate'
    )
    gnss_pos_recovery_weight_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_max_abs_vertical_speed_mps', default_value='0.35',
        description='Maximum absolute vertical speed for GNSS position recovery motion gate'
    )
    gnss_pos_recovery_weight_persistence_updates = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_persistence_updates', default_value='3',
        description='Consecutive GNSS updates required before recovery hold is armed'
    )
    gnss_pos_recovery_weight_hold_sec = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_hold_sec', default_value='3.0',
        description='Seconds to hold recovery after persistence is reached'
    )
    gnss_pos_recovery_weight_attack_sec = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_attack_sec', default_value='0.5',
        description='GNSS position recovery std attack time constant'
    )
    gnss_pos_recovery_weight_decay_sec = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_decay_sec', default_value='2.0',
        description='GNSS position recovery std decay time constant'
    )
    gnss_pos_recovery_weight_std_min_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_std_min_h_m', default_value='0.12',
        description='Minimum recovery horizontal GNSS position std while held'
    )
    gnss_pos_recovery_weight_std_max_h_m = DeclareLaunchArgument(
        'gnss_pos_recovery_weight_std_max_h_m', default_value='0.16',
        description='Maximum recovery horizontal GNSS position std'
    )
    context_gnss_pos_floor_enable = DeclareLaunchArgument(
        'context_gnss_pos_floor_enable', default_value='false',
        description='Enable default-off context scheduled GNSS horizontal position std floor'
    )
    context_gnss_pos_floor_apply_mission = DeclareLaunchArgument(
        'context_gnss_pos_floor_apply_mission', default_value='true',
        description='Allow context GNSS position std floor during AUTO.MISSION'
    )
    context_gnss_pos_floor_apply_rtl = DeclareLaunchArgument(
        'context_gnss_pos_floor_apply_rtl', default_value='false',
        description='Allow context GNSS position std floor during AUTO.RTL'
    )
    context_gnss_pos_floor_mission_base_h_m = DeclareLaunchArgument(
        'context_gnss_pos_floor_mission_base_h_m', default_value='0.08',
        description='Mission base horizontal GNSS position std floor'
    )
    context_gnss_pos_floor_turn_post_h_m = DeclareLaunchArgument(
        'context_gnss_pos_floor_turn_post_h_m', default_value='0.10',
        description='Turning and post-turn horizontal GNSS position std floor'
    )
    context_gnss_pos_floor_armed_cruise_h_m = DeclareLaunchArgument(
        'context_gnss_pos_floor_armed_cruise_h_m', default_value='0.09',
        description='Armed-cruise horizontal GNSS position std floor'
    )
    context_gnss_pos_floor_rtl_h_m = DeclareLaunchArgument(
        'context_gnss_pos_floor_rtl_h_m', default_value='0.08',
        description='RTL horizontal GNSS position std floor if RTL application is enabled'
    )
    context_gnss_pos_floor_attack_sec = DeclareLaunchArgument(
        'context_gnss_pos_floor_attack_sec', default_value='0.5',
        description='Context GNSS position std floor attack time constant'
    )
    context_gnss_pos_floor_decay_sec = DeclareLaunchArgument(
        'context_gnss_pos_floor_decay_sec', default_value='1.5',
        description='Context GNSS position std floor decay time constant'
    )
    heading_cruise_micro_track_enable = DeclareLaunchArgument(
        'heading_cruise_micro_track_enable', default_value='false',
        description='Enable default-off low-gain cruise micro heading tracking during AUTO.MISSION'
    )
    heading_cruise_micro_track_std_deg = DeclareLaunchArgument(
        'heading_cruise_micro_track_std_deg', default_value='2.0',
        description='Heading std used by cruise micro heading tracking'
    )
    heading_cruise_micro_track_min_residual_deg = DeclareLaunchArgument(
        'heading_cruise_micro_track_min_residual_deg', default_value='0.15',
        description='Minimum absolute heading residual for cruise micro heading tracking'
    )
    heading_cruise_micro_track_max_residual_deg = DeclareLaunchArgument(
        'heading_cruise_micro_track_max_residual_deg', default_value='1.0',
        description='Maximum absolute heading residual for cruise micro heading tracking'
    )
    heading_cruise_micro_track_min_horizontal_speed_mps = DeclareLaunchArgument(
        'heading_cruise_micro_track_min_horizontal_speed_mps', default_value='4.5',
        description='Minimum horizontal speed for cruise micro heading tracking'
    )
    heading_cruise_micro_track_max_vertical_speed_mps = DeclareLaunchArgument(
        'heading_cruise_micro_track_max_vertical_speed_mps', default_value='0.35',
        description='Maximum absolute vertical speed for cruise micro heading tracking'
    )
    heading_cruise_micro_track_max_gyro_deg_s = DeclareLaunchArgument(
        'heading_cruise_micro_track_max_gyro_deg_s', default_value='3.5',
        description='Maximum gyro norm for cruise micro heading tracking'
    )
    heading_cruise_micro_track_max_source_yaw_rate_deg_s = DeclareLaunchArgument(
        'heading_cruise_micro_track_max_source_yaw_rate_deg_s', default_value='1.0',
        description='Maximum source heading yaw rate for cruise micro heading tracking'
    )
    heading_yaw_gain_hygiene_enable = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_enable', default_value='false',
        description='Enable default-off HVE1a yaw covariance reopen before eligible heading updates'
    )
    heading_yaw_gain_hygiene_apply_mission = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_mission', default_value='true',
        description='Allow HVE1a heading yaw-gain hygiene during AUTO.MISSION'
    )
    heading_yaw_gain_hygiene_apply_rtl = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_rtl', default_value='false',
        description='Allow HVE1a heading yaw-gain hygiene during AUTO.RTL'
    )
    heading_yaw_gain_hygiene_apply_armed_cruise = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_armed_cruise', default_value='true',
        description='Allow HVE1a on armed_cruise_track heading updates'
    )
    heading_yaw_gain_hygiene_apply_post_turn = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_post_turn', default_value='true',
        description='Allow HVE1a on post-turn heading updates'
    )
    heading_yaw_gain_hygiene_apply_turn = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_turn', default_value='false',
        description='Allow HVE1a on turn_track heading updates'
    )
    heading_yaw_gain_hygiene_apply_update = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_apply_update', default_value='false',
        description='Allow HVE1a on plain update/cruise_micro_track heading updates'
    )
    heading_yaw_gain_hygiene_yaw_std_floor_deg = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_yaw_std_floor_deg', default_value='1.0',
        description='Target yaw covariance std floor for HVE1a'
    )
    heading_yaw_gain_hygiene_min_residual_deg = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_min_residual_deg', default_value='0.5',
        description='Minimum absolute heading residual for HVE1a'
    )
    heading_yaw_gain_hygiene_max_residual_deg = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_max_residual_deg', default_value='2.0',
        description='Maximum absolute heading residual for HVE1a; <=0 disables upper gate'
    )
    heading_yaw_gain_hygiene_min_horizontal_speed_mps = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_min_horizontal_speed_mps', default_value='3.0',
        description='Minimum horizontal speed for HVE1a'
    )
    heading_yaw_gain_hygiene_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_max_abs_vertical_speed_mps', default_value='0.5',
        description='Maximum absolute vertical speed for HVE1a'
    )
    heading_yaw_gain_hygiene_max_gyro_deg_s = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_max_gyro_deg_s', default_value='8.0',
        description='Maximum gyro norm for HVE1a'
    )
    heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s', default_value='3.0',
        description='Maximum source heading yaw rate for HVE1a'
    )
    heading_yaw_gain_hygiene_max_rate_hz = DeclareLaunchArgument(
        'heading_yaw_gain_hygiene_max_rate_hz', default_value='5.0',
        description='Maximum HVE1a yaw covariance reopen rate in Hz; 0 allows every eligible update'
    )
    armed_cruise_native_gnss_vel_min_horizontal_speed_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_min_horizontal_speed_mps', default_value='0.5',
        description='Minimum horizontal speed for late-cruise native GNSS velocity tightening'
    )
    armed_cruise_native_gnss_vel_std_h_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_std_h_mps', default_value='0.05',
        description='Horizontal native GNSS velocity std used when late-cruise override is active'
    )
    armed_cruise_native_gnss_vel_std_u_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_std_u_mps', default_value='0.10',
        description='Vertical native GNSS velocity std used when late-cruise override is active'
    )
    armed_cruise_native_gnss_vel_residual_boost_enable = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_residual_boost_enable', default_value='true',
        description='Temporarily tighten native GNSS velocity std when recent cruise east-velocity residual stays large'
    )
    armed_cruise_native_gnss_vel_residual_boost_threshold_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_residual_boost_threshold_mps', default_value='0.10',
        description='Previous GNSS east-velocity residual threshold that arms the temporary cruise velocity-std boost'
    )
    armed_cruise_native_gnss_vel_residual_boost_hold_sec = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_residual_boost_hold_sec', default_value='8.0',
        description='How long to keep the temporary cruise velocity-std boost active after a large east residual hit'
    )
    armed_cruise_native_gnss_vel_residual_boost_std_h_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_residual_boost_std_h_mps', default_value='0.03',
        description='Horizontal native GNSS velocity std used while the temporary residual boost is active'
    )
    armed_cruise_native_gnss_vel_residual_boost_std_u_mps = DeclareLaunchArgument(
        'armed_cruise_native_gnss_vel_residual_boost_std_u_mps', default_value='0.08',
        description='Vertical native GNSS velocity std used while the temporary residual boost is active'
    )
    native_gnss_velocity_outlier_guard_enable = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_enable', default_value='false',
        description='Enable default-off native GNSS velocity outlier guard before velocity updates'
    )
    native_gnss_velocity_outlier_guard_apply_mission = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_apply_mission', default_value='true',
        description='Allow native GNSS velocity outlier guard during AUTO.MISSION'
    )
    native_gnss_velocity_outlier_guard_apply_rtl = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_apply_rtl', default_value='false',
        description='Allow native GNSS velocity outlier guard during AUTO.RTL'
    )
    native_gnss_velocity_outlier_guard_speed_mismatch_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_speed_mismatch_mps', default_value='0.60',
        description='Skip native velocity update when native horizontal speed differs from MAVROS speed by this much'
    )
    native_gnss_velocity_outlier_guard_core_residual_h_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_core_residual_h_mps', default_value='0.80',
        description='Skip native velocity update when native-vs-core horizontal velocity residual exceeds this threshold'
    )
    native_gnss_velocity_outlier_guard_min_horizontal_speed_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_min_horizontal_speed_mps', default_value='1.0',
        description='Minimum horizontal speed for native velocity outlier guard'
    )
    native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps', default_value='0.5',
        description='Maximum absolute vertical speed for native velocity outlier guard'
    )
    native_gnss_velocity_outlier_guard_apply_turning_context = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_apply_turning_context', default_value='true',
        description='Allow native velocity outlier guard during active turning context'
    )
    native_gnss_velocity_outlier_guard_action = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_action', default_value='skip',
        description='Action for native velocity outlier guard hits: skip or reweight'
    )
    native_gnss_velocity_outlier_guard_reweight_std_h_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_reweight_std_h_mps', default_value='0.12',
        description='Horizontal velocity std used when guard action is reweight'
    )
    native_gnss_velocity_outlier_guard_reweight_std_u_mps = DeclareLaunchArgument(
        'native_gnss_velocity_outlier_guard_reweight_std_u_mps', default_value='0.20',
        description='Vertical velocity std used when guard action is reweight'
    )
    native_gnss_velocity_low_speed_turn_source_guard_enable = DeclareLaunchArgument(
        'native_gnss_velocity_low_speed_turn_source_guard_enable', default_value='false',
        description='Enable narrow low-speed-turn native/core velocity source mismatch guard'
    )
    native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps = DeclareLaunchArgument(
        'native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps', default_value='2.2',
        description='Maximum MAVROS horizontal speed for low-speed-turn source mismatch guard'
    )
    native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s = DeclareLaunchArgument(
        'native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s', default_value='15.0',
        description='Minimum IMU gyro norm for low-speed-turn source mismatch guard'
    )
    native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps = DeclareLaunchArgument(
        'native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps', default_value='1.0',
        description='Native-vs-core horizontal velocity residual threshold for low-speed-turn source mismatch guard'
    )
    armed_cruise_vertical_cov_reopen_enable = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_enable', default_value='true',
        description='Temporarily reopen vertical covariance when late-cruise vertical residuals stay large'
    )
    armed_cruise_vertical_cov_reopen_threshold_m = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_threshold_m', default_value='0.15',
        description='Vertical residual threshold that arms the temporary vertical covariance reopen'
    )
    armed_cruise_vertical_cov_reopen_hold_sec = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_hold_sec', default_value='8.0',
        description='How long to keep the temporary vertical covariance reopen active'
    )
    armed_cruise_vertical_cov_reopen_pos_std_m = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_pos_std_m', default_value='0.15',
        description='Vertical position std floor while the temporary covariance reopen is active'
    )
    armed_cruise_vertical_cov_reopen_vel_std_mps = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_vel_std_mps', default_value='0.10',
        description='Vertical velocity std floor while the temporary covariance reopen is active'
    )
    armed_cruise_vertical_cov_reopen_accbias_std_z_mps2 = DeclareLaunchArgument(
        'armed_cruise_vertical_cov_reopen_accbias_std_z_mps2', default_value='0.05',
        description='Vertical accelerometer bias std floor while the temporary covariance reopen is active'
    )
    post_flight_vertical_cov_reopen_enable = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_enable', default_value='true',
        description='Temporarily reopen vertical covariance after landing while disarmed/post-flight vertical drift is being pulled back'
    )
    post_flight_vertical_cov_reopen_threshold_m = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_threshold_m', default_value='0.12',
        description='Vertical residual threshold that refreshes the post-flight vertical covariance reopen'
    )
    post_flight_vertical_cov_reopen_hold_sec = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_hold_sec', default_value='20.0',
        description='How long to keep the post-flight vertical covariance reopen active after each qualifying trigger'
    )
    post_flight_vertical_cov_reopen_grace_sec = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_grace_sec', default_value='30.0',
        description='Always keep the post-flight vertical covariance reopen active for this long immediately after disarm'
    )
    post_flight_vertical_cov_reopen_pos_std_m = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_pos_std_m', default_value='0.25',
        description='Vertical position std floor while the post-flight covariance reopen is active'
    )
    post_flight_vertical_cov_reopen_vel_std_mps = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_vel_std_mps', default_value='0.10',
        description='Vertical velocity std floor while the post-flight covariance reopen is active'
    )
    post_flight_vertical_cov_reopen_accbias_std_z_mps2 = DeclareLaunchArgument(
        'post_flight_vertical_cov_reopen_accbias_std_z_mps2', default_value='0.05',
        description='Vertical accelerometer bias std floor while the post-flight covariance reopen is active'
    )
    terminal_descent_observation_enable = DeclareLaunchArgument(
        'terminal_descent_observation_enable', default_value='false',
        description='Enable narrow observation tightening in armed RTL/landing terminal descent'
    )
    terminal_descent_require_rtl_mode = DeclareLaunchArgument(
        'terminal_descent_require_rtl_mode', default_value='true',
        description='Require MAVROS mode to contain RTL or LAND before terminal descent tightening can activate'
    )
    terminal_descent_max_horizontal_speed_mps = DeclareLaunchArgument(
        'terminal_descent_max_horizontal_speed_mps', default_value='1.6',
        description='Maximum horizontal speed for terminal descent tightening'
    )
    terminal_descent_min_vertical_speed_mps = DeclareLaunchArgument(
        'terminal_descent_min_vertical_speed_mps', default_value='0.30',
        description='Minimum absolute vertical speed for terminal descent tightening'
    )
    terminal_descent_max_gyro_deg_s = DeclareLaunchArgument(
        'terminal_descent_max_gyro_deg_s', default_value='30.0',
        description='Maximum IMU gyro norm for terminal descent tightening; <=0 disables this gate'
    )
    terminal_descent_max_source_yaw_rate_deg_s = DeclareLaunchArgument(
        'terminal_descent_max_source_yaw_rate_deg_s', default_value='30.0',
        description='Maximum source yaw rate for terminal descent tightening; <=0 disables this gate'
    )
    terminal_descent_min_armed_time_sec = DeclareLaunchArgument(
        'terminal_descent_min_armed_time_sec', default_value='0.0',
        description='Minimum armed duration before terminal descent tightening can activate'
    )
    terminal_descent_native_gnss_vel_override_enable = DeclareLaunchArgument(
        'terminal_descent_native_gnss_vel_override_enable', default_value='true',
        description='Tighten native GNSS velocity std during terminal descent context'
    )
    terminal_descent_native_gnss_vel_std_h_mps = DeclareLaunchArgument(
        'terminal_descent_native_gnss_vel_std_h_mps', default_value='0.03',
        description='Horizontal native GNSS velocity std used in terminal descent context'
    )
    terminal_descent_native_gnss_vel_std_u_mps = DeclareLaunchArgument(
        'terminal_descent_native_gnss_vel_std_u_mps', default_value='0.05',
        description='Vertical native GNSS velocity std used in terminal descent context'
    )
    terminal_descent_vertical_cov_reopen_enable = DeclareLaunchArgument(
        'terminal_descent_vertical_cov_reopen_enable', default_value='true',
        description='Reopen vertical covariance while terminal descent context is active'
    )
    terminal_descent_vertical_cov_reopen_pos_std_m = DeclareLaunchArgument(
        'terminal_descent_vertical_cov_reopen_pos_std_m', default_value='0.15',
        description='Vertical position std floor while terminal descent covariance reopen is active'
    )
    terminal_descent_vertical_cov_reopen_vel_std_mps = DeclareLaunchArgument(
        'terminal_descent_vertical_cov_reopen_vel_std_mps', default_value='0.05',
        description='Vertical velocity std floor while terminal descent covariance reopen is active'
    )
    terminal_descent_vertical_cov_reopen_accbias_std_z_mps2 = DeclareLaunchArgument(
        'terminal_descent_vertical_cov_reopen_accbias_std_z_mps2', default_value='0.05',
        description='Vertical accelerometer bias std floor while terminal descent covariance reopen is active'
    )
    terminal_descent_horizontal_zero_vel_enable = DeclareLaunchArgument(
        'terminal_descent_horizontal_zero_vel_enable', default_value='false',
        description='Inject a horizontal zero-velocity observation in low-horizontal-speed terminal descent'
    )
    terminal_descent_horizontal_zero_vel_max_hspeed_mps = DeclareLaunchArgument(
        'terminal_descent_horizontal_zero_vel_max_hspeed_mps', default_value='0.30',
        description='Maximum horizontal speed for terminal descent horizontal zero-velocity observation'
    )
    terminal_descent_horizontal_zero_vel_std_h_mps = DeclareLaunchArgument(
        'terminal_descent_horizontal_zero_vel_std_h_mps', default_value='0.05',
        description='Horizontal velocity std for terminal descent horizontal zero-velocity observation'
    )
    terminal_descent_horizontal_zero_vel_std_u_mps = DeclareLaunchArgument(
        'terminal_descent_horizontal_zero_vel_std_u_mps', default_value='10.0',
        description='Vertical velocity std for terminal descent horizontal zero-velocity observation'
    )
    tilt_force_relock_min_residual_deg = DeclareLaunchArgument(
        'tilt_force_relock_min_residual_deg', default_value='2.0',
        description='Minimum roll/pitch residual for tilt force relock'
    )
    tilt_force_relock_roll_pitch_std_deg = DeclareLaunchArgument(
        'tilt_force_relock_roll_pitch_std_deg', default_value='1.5',
        description='Roll/pitch std for tilt force relock'
    )
    tilt_force_relock_once_per_motion_context = DeclareLaunchArgument(
        'tilt_force_relock_once_per_motion_context', default_value='true',
        description='Limit tilt force relock to one hit per post-turn or armed-cruise context'
    )
    tilt_force_relock_max_rate_hz = DeclareLaunchArgument(
        'tilt_force_relock_max_rate_hz', default_value='2.0',
        description='Maximum rate for tilt force relock'
    )
    aligned_fallback_raw = DeclareLaunchArgument(
        'aligned_fallback_raw', default_value='false',
        description='Publish raw IEKF on /iekf/state_aligned before alignment is ready'
    )
    ekf2_use_input_stamp = DeclareLaunchArgument(
        'ekf2_use_input_stamp', default_value='true',
        description='Use input stamp for EKF2 relay; false stamps relayed samples with node time'
    )
    ekf2_relay_publish_pose = DeclareLaunchArgument(
        'ekf2_relay_publish_pose', default_value='true',
        description='Publish /ekf2/pose from ekf2_state_relay in addition to /ekf2/pose_odom'
    )

    imu_use_node_stamp = DeclareLaunchArgument(
        'imu_use_node_stamp', default_value='false',
        description='Use /clock stamp for IMU output (false uses MAVROS stamp)'
    )
    enable_imu_converter = DeclareLaunchArgument(
        'enable_imu_converter', default_value='false',
        description='Run imu_flu_to_frd.py only for diagnostics; KF-GINS subscribes raw IMU directly'
    )

    # ============ MAVROS topics (可覆盖) ============
    # 注意：MAVROS 的 ROS 命名空间由其 launch 参数 `namespace` 控制（默认是 `/mavros`）。
    # 终端里看到的 “UAS Prefix: /uas1” 不是 ROS topic 前缀。
    # 如果你想让 MAVROS topics 变成 `/uas1/mavros/...`，需要这样启动 MAVROS：
    #   ros2 launch mavros px4.launch namespace:=uas1/mavros ...
    mavros_imu_topic = DeclareLaunchArgument(
        'mavros_imu_topic', default_value='/mavros/imu/data_raw',
        description='MAVROS IMU topic (default: /mavros/imu/data_raw)'
    )
    imu_source = DeclareLaunchArgument(
        'imu_source', default_value='mavros_raw',
        description='IMU source for KF-GINS: mavros_raw, px4_sensor_combined, or px4_vehicle_imu'
    )
    px4_sensor_combined_topic = DeclareLaunchArgument(
        'px4_sensor_combined_topic', default_value='/fmu/out/sensor_combined',
        description='PX4 DDS SensorCombined topic'
    )
    px4_vehicle_imu_topic = DeclareLaunchArgument(
        'px4_vehicle_imu_topic', default_value='/fmu/out/vehicle_imu',
        description='PX4 DDS VehicleImu topic'
    )
    px4_imu_qos_depth = DeclareLaunchArgument(
        'px4_imu_qos_depth', default_value='200',
        description='Queue depth for PX4 DDS IMU subscriptions in kf_gins_node'
    )
    comparison_subscribe_ekf2_pose = DeclareLaunchArgument(
        'comparison_subscribe_ekf2_pose', default_value='true',
        description='Subscribe real_time_comparison to /ekf2/pose in addition to /ekf2/pose_odom'
    )
    mavros_gps_topic = DeclareLaunchArgument(
        'mavros_gps_topic', default_value='/mavros/global_position/raw/fix',
        description='MAVROS GNSS topic (default: /mavros/global_position/raw/fix)'
    )
    gnss_relay_mode = DeclareLaunchArgument(
        'gnss_relay_mode', default_value='mavros',
        description='GNSS relay source: mavros, px4_sensor_gps, px4_vehicle_global_position, or disabled'
    )
    enable_gnss_relay = DeclareLaunchArgument(
        'enable_gnss_relay', default_value='true',
        description='Enable GNSS relay nodes. False disables both mavros-source and PX4-source GNSS relays.'
    )
    gnss_source = DeclareLaunchArgument(
        'gnss_source', default_value='navsatfix',
        description='KF-GINS GNSS source: navsatfix, px4_sensor_gps, or px4_vehicle_global_position'
    )
    px4_sensor_gps_topic = DeclareLaunchArgument(
        'px4_sensor_gps_topic', default_value='/fmu/out/vehicle_gps_position',
        description='PX4 DDS SensorGps topic'
    )
    px4_vehicle_global_position_topic = DeclareLaunchArgument(
        'px4_vehicle_global_position_topic', default_value='/fmu/out/vehicle_global_position',
        description='PX4 DDS VehicleGlobalPosition topic'
    )
    mavros_local_pose_topic = DeclareLaunchArgument(
        'mavros_local_pose_topic', default_value='/mavros/local_position/pose',
        description='MAVROS local pose topic (default: /mavros/local_position/pose)'
    )
    ekf2_input_mode = DeclareLaunchArgument(
        'ekf2_input_mode', default_value='mavros_pose',
        description='EKF2 relay input mode: mavros_pose or px4_vehicle_odometry'
    )
    px4_vehicle_odometry_topic = DeclareLaunchArgument(
        'px4_vehicle_odometry_topic', default_value='/fmu/out/vehicle_odometry',
        description='PX4 DDS VehicleOdometry topic for EKF2 relay'
    )
    enable_px4_aux_state_relay = DeclareLaunchArgument(
        'enable_px4_aux_state_relay', default_value='false',
        description='Relay PX4 VehicleOdometry into MAVROS-like helper topics for KF-GINS speed/heading'
    )
    px4_aux_imu_topic = DeclareLaunchArgument(
        'px4_aux_imu_topic', default_value='/px4_aux/imu/data',
        description='Relay output IMU topic used as KF-GINS heading source'
    )
    px4_aux_local_velocity_topic = DeclareLaunchArgument(
        'px4_aux_local_velocity_topic', default_value='/px4_aux/local_position/velocity_local',
        description='Relay output local velocity topic used as KF-GINS speed source'
    )
    mavros_local_velocity_topic = DeclareLaunchArgument(
        'mavros_local_velocity_topic', default_value='/mavros/local_position/velocity_local',
        description='MAVROS local velocity topic (default: /mavros/local_position/velocity_local)'
    )
    speed_source = DeclareLaunchArgument(
        'speed_source', default_value='mavros_local_velocity',
        description='Speed source for KF-GINS heading gates: mavros_local_velocity, px4_vehicle_local_position, or px4_vehicle_odometry'
    )
    px4_vehicle_local_position_topic = DeclareLaunchArgument(
        'px4_vehicle_local_position_topic', default_value='/fmu/out/vehicle_local_position',
        description='PX4 DDS VehicleLocalPosition topic'
    )
    mavros_state_topic = DeclareLaunchArgument(
        'mavros_state_topic', default_value='/mavros/state',
        description='MAVROS state topic (default: /mavros/state)'
    )
    heading_source = DeclareLaunchArgument(
        'heading_source', default_value='mavros_imu',
        description='Heading source for KF-GINS: mavros_imu, px4_vehicle_attitude, or px4_vehicle_odometry'
    )
    mavros_heading_topic = DeclareLaunchArgument(
        'mavros_heading_topic', default_value='/mavros/imu/data',
        description='MAVROS EKF2 heading topic (default: /mavros/imu/data)'
    )
    px4_vehicle_attitude_topic = DeclareLaunchArgument(
        'px4_vehicle_attitude_topic', default_value='/fmu/out/vehicle_attitude',
        description='PX4 DDS VehicleAttitude topic'
    )
    mavros_hil_gps_topic = DeclareLaunchArgument(
        'mavros_hil_gps_topic', default_value='/mavros/hil/gps',
        description='MAVROS HIL GPS topic (default: /mavros/hil/gps)'
    )
    gnss_relay_start_delay_sec = DeclareLaunchArgument(
        'gnss_relay_start_delay_sec', default_value='0.0',
        description='Delay GNSS relay startup so KF-GINS can receive heading before first GNSS reset'
    )
    gnss_relay_publish_enable = DeclareLaunchArgument(
        'gnss_relay_publish_enable', default_value='true',
        description='Enable /gps/fix publishing in mavros-source gnss_relay.py; false keeps subscription-only'
    )
    gnss_relay_subscribe_enable = DeclareLaunchArgument(
        'gnss_relay_subscribe_enable', default_value='true',
        description='Enable MAVROS GNSS subscription in mavros-source gnss_relay.py; false keeps process-only'
    )
    gnss_relay_publish_min_interval_sec = DeclareLaunchArgument(
        'gnss_relay_publish_min_interval_sec', default_value='0.0',
        description='Minimum PX4-source GNSS relay publish interval in seconds; 0 disables downsampling'
    )

    gnss_disturbance_enable = DeclareLaunchArgument(
        'gnss_disturbance_enable', default_value='false',
        description='Enable deterministic trace-based GNSS disturbance replay in px4_gnss_relay.py'
    )
    gnss_disturbance_trace_csv = DeclareLaunchArgument(
        'gnss_disturbance_trace_csv', default_value='',
        description='CSV trace path for deterministic GNSS disturbance replay'
    )
    gnss_disturbance_log_csv = DeclareLaunchArgument(
        'gnss_disturbance_log_csv', default_value='',
        description='Optional CSV log path for applied GNSS disturbance replay'
    )
    gnss_disturbance_time_offset_sec = DeclareLaunchArgument(
        'gnss_disturbance_time_offset_sec', default_value='0.0',
        description='Time offset subtracted from trace elapsed time; positive values delay trace replay'
    )

    # ============ GNSS Dropzones (可选) ============
    # 用于模拟 GNSS 遮挡/丢星：当飞机进入配置的盒子区域时，将 /gps/fix 发布为 NO_FIX（或 stop/nan）
    enable_gps_dropzones = DeclareLaunchArgument(
        'enable_gps_dropzones', default_value='false',
        description='Enable GNSS dropzones relay (replace gnss_relay.py)'
    )
    inject_dropzone_gps_to_px4 = DeclareLaunchArgument(
        'inject_dropzone_gps_to_px4', default_value='true',
        description='Inject dropzoned /gps/fix into PX4 via MAVROS'
    )
    px4_gps_injection_mode = DeclareLaunchArgument(
        'px4_gps_injection_mode', default_value='hil_gps',
        description='PX4 GPS injection mode when dropzones enabled: hil_gps or gps_input'
    )
    px4_set_params = DeclareLaunchArgument(
        'px4_set_params', default_value='true',
        description='Auto-set PX4 params for GPS injection (e.g. MAV_USEHILGPS)'
    )
    px4_param_service_v2 = DeclareLaunchArgument(
        'px4_param_service_v2', default_value='/mavros/param/set_v2',
        description='MAVROS ParamSetV2 service'
    )
    px4_param_service = DeclareLaunchArgument(
        'px4_param_service', default_value='/mavros/param/set',
        description='MAVROS ParamSet service (fallback)'
    )

    # 【修复】仿真 GNSS 自适应参数声明
    use_sim_gnss_std = DeclareLaunchArgument(
        'use_sim_gnss_std', default_value='true',
        description='Enable simulation-specific GNSS std (use 0.1m instead of 5.0m)'
    )
    
    sim_gnss_std_h_m = DeclareLaunchArgument(
        'sim_gnss_std_h_m', default_value='0.08',
        description='Simulation GNSS horizontal std (meters)'
    )
    
    sim_gnss_std_u_m = DeclareLaunchArgument(
        'sim_gnss_std_u_m', default_value='0.10',
        description='Simulation GNSS vertical std (meters)'
    )

    gnss_position_lag_compensation_enable = DeclareLaunchArgument(
        'gnss_position_lag_compensation_enable', default_value='false',
        description='Enable runtime GNSS position source-lag compensation'
    )

    gnss_position_lag_compensation_sec = DeclareLaunchArgument(
        'gnss_position_lag_compensation_sec', default_value='0.25',
        description='GNSS position source-lag compensation horizon (seconds)'
    )

    gnss_position_lag_compensation_max_sec = DeclareLaunchArgument(
        'gnss_position_lag_compensation_max_sec', default_value='0.50',
        description='Maximum allowed GNSS position source-lag compensation horizon (seconds)'
    )

    gnss_position_lag_compensation_min_speed_mps = DeclareLaunchArgument(
        'gnss_position_lag_compensation_min_speed_mps', default_value='0.50',
        description='Minimum native GNSS horizontal speed for source-lag compensation'
    )

    aligned_path_require_armed = DeclareLaunchArgument(
        'aligned_path_require_armed', default_value='true',
        description='Require arming before publishing IEKF aligned path'
    )

    enable_ekf2_path = DeclareLaunchArgument(
        'enable_ekf2_path', default_value='true',
        description='Enable EKF2 path publisher'
    )
    enable_iekf_path = DeclareLaunchArgument(
        'enable_iekf_path', default_value='true',
        description='Enable raw IEKF path publisher'
    )
    enable_gt_path = DeclareLaunchArgument(
        'enable_gt_path', default_value='true',
        description='Enable ground-truth/odom-to-path publisher'
    )
    enable_ekf2_relay = DeclareLaunchArgument(
        'enable_ekf2_relay', default_value='true',
        description='Enable EKF2 pose relay node'
    )
    enable_kf_gins = DeclareLaunchArgument(
        'enable_kf_gins', default_value='true',
        description='Enable KF-GINS node'
    )
    kf_gins_start_delay_sec = DeclareLaunchArgument(
        'kf_gins_start_delay_sec', default_value='0.0',
        description='Delay KF-GINS startup to let helper topics publish before first GNSS reset'
    )
    enable_iekf_aligned_path = DeclareLaunchArgument(
        'enable_iekf_aligned_path', default_value='true',
        description='Enable aligned IEKF path publisher'
    )
    enable_static_tf = DeclareLaunchArgument(
        'enable_static_tf', default_value='true',
        description='Enable world->map static transform publisher'
    )

    gps_dropzones_yaml = PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'),
        'config',
        'gps_dropzones.yaml'
    ])
    
    # ============ 静态 TF ============
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_static_tf')),
    )

    # ============ 数据预处理 ============
    imu_convert = Node(
        package='kf_gins_ros2_native',
        executable='imu_flu_to_frd.py',
        name='imu_flu_to_frd',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('mavros_imu_topic'),
            'output_topic': '/imu/data',
            'flip_gravity': True,
            # 默认用 MAVROS stamp；若 /clock 稳定可切换为 use_node_stamp=true
            'use_node_stamp': LaunchConfiguration('imu_use_node_stamp'),
            # 当 /clock 不前进时回退到输入时间戳，保证单调
            'fallback_to_input_stamp': True,
            'force_monotonic_stamp': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_imu_converter'))
    )

    px4_aux_state_relay = Node(
        package='kf_gins_ros2_native',
        executable='px4_aux_state_relay.py',
        name='px4_aux_state_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_odometry_topic': LaunchConfiguration('px4_vehicle_odometry_topic'),
            'imu_output_topic': LaunchConfiguration('px4_aux_imu_topic'),
            'velocity_output_topic': LaunchConfiguration('px4_aux_local_velocity_topic'),
            'use_px4_stamp': True,
            'imu_frame_id': 'base_link',
            'velocity_frame_id': 'map',
        }],
        condition=IfCondition(LaunchConfiguration('enable_px4_aux_state_relay')),
    )

    gnss_relay = Node(
        package='kf_gins_ros2_native',
        executable='gnss_relay.py',
        name='gnss_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_topic': LaunchConfiguration('mavros_gps_topic'),
            'out_topic': '/gps/fix',
            'subscribe_enable': ParameterValue(LaunchConfiguration('gnss_relay_subscribe_enable'), value_type=bool),
            'publish_enable': ParameterValue(LaunchConfiguration('gnss_relay_publish_enable'), value_type=bool),
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gnss_relay'),
            "' == 'true' and '",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'false' and '",
            LaunchConfiguration('gnss_relay_mode'),
            "' == 'mavros'",
        ])),
    )
    px4_gnss_relay = Node(
        package='kf_gins_ros2_native',
        executable='px4_gnss_relay.py',
        name='px4_gnss_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_mode': PythonExpression([
                "'sensor_gps' if '",
                LaunchConfiguration('gnss_relay_mode'),
                "' == 'px4_sensor_gps' else 'vehicle_global_position'"
            ]),
            'sensor_gps_topic': LaunchConfiguration('px4_sensor_gps_topic'),
            'vehicle_global_position_topic': LaunchConfiguration('px4_vehicle_global_position_topic'),
            'output_topic': PythonExpression([
                "'/gps/dropzone_input' if '",
                LaunchConfiguration('enable_gps_dropzones'),
                "' == 'true' else '/gps/fix'"
            ]),
            'disturbance_enable': ParameterValue(LaunchConfiguration('gnss_disturbance_enable'), value_type=bool),
            'disturbance_trace_csv': LaunchConfiguration('gnss_disturbance_trace_csv'),
            'disturbance_log_csv': LaunchConfiguration('gnss_disturbance_log_csv'),
            'disturbance_time_offset_sec': ParameterValue(LaunchConfiguration('gnss_disturbance_time_offset_sec'), value_type=float),
            'publish_min_interval_sec': ParameterValue(LaunchConfiguration('gnss_relay_publish_min_interval_sec'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gnss_relay'),
            "' == 'true' and ('",
            LaunchConfiguration('gnss_relay_mode'),
            "' == 'px4_sensor_gps' or '",
            LaunchConfiguration('gnss_relay_mode'),
            "' == 'px4_vehicle_global_position')",
        ])),
    )
    gnss_relay_delayed = TimerAction(
        period=LaunchConfiguration('gnss_relay_start_delay_sec'),
        actions=[gnss_relay, px4_gnss_relay],
    )

    gps_dropzones = Node(
        package='kf_gins_ros2_native',
        executable='gps_relay_dropzones.py',
        name='gps_relay_dropzones',
        output='screen',
        parameters=[
            gps_dropzones_yaml,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'in_fix_topic': PythonExpression([
                    "'",
                    LaunchConfiguration('mavros_gps_topic'),
                    "' if '",
                    LaunchConfiguration('gnss_relay_mode'),
                    "' == 'mavros' else '/gps/dropzone_input'"
                ]),
                'pose_topic': LaunchConfiguration('mavros_local_pose_topic'),
                'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
                'out_fix_topic': '/gps/fix',
                # Disarmed 时不做遮挡，避免 IEKF 在地面纯惯导发散
                'require_armed_for_drop': True,
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_gps_dropzones')),
    )

    px4_param_setter = Node(
        package='kf_gins_ros2_native',
        executable='px4_param_setter.py',
        name='px4_param_setter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'param_ints': ['MAV_USEHILGPS=1'],
            'service_set_v2': LaunchConfiguration('px4_param_service_v2'),
            'service_set': LaunchConfiguration('px4_param_service'),
            'wait_timeout_sec': 3.0,
            'retry_count': 8,
            'retry_sleep_sec': 1.0,
            'force_set': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_set_params'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'hil_gps'",
        ])),
    )

    hil_gps_relay = Node(
        package='kf_gins_ros2_native',
        executable='gps_fix_to_mavros_hil_gps.py',
        name='hil_gps_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_fix_topic': '/gps/fix',
            'out_hil_gps_topic': LaunchConfiguration('mavros_hil_gps_topic'),
            'use_input_stamp': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'hil_gps'",
        ])),
    )

    gps_input_relay = Node(
        package='kf_gins_ros2_native',
        executable='gps_fix_to_mavros_gps_input.py',
        name='gps_input_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_fix_topic': '/gps/fix',
            'out_gps_input_topic': '/mavros/gps_input',
            'use_input_stamp': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'gps_input'",
        ])),
    )

    # ============ 轨迹发布器 ============
    # EKF2 路径转换
    ekf2_path = Node(
        package='kf_gins_ros2_native',
        executable='ekf2_path_publisher.py',
        name='ekf2_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 默认 1000 点大约只有几十秒，会出现“尾迹滚动/老轨迹消失”的错觉
            'max_path_length': 20000,
            'min_distance': 0.05,
            'require_armed': True,
            'clear_on_arm_transition': False,
            'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
            'startup_ignore_sec': 3.0,
            'max_abs_position_m': 10000.0,
            'max_jump_distance_m': 50.0,
            'mavros_state_stale_sec': 1.5,
            'armed_false_hold_sec': 6.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_ekf2_path')),
    )

    iekf_path = Node(
        package='kf_gins_ros2_native',
        executable='iekf_path_publisher.py',
        name='iekf_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/kf_gins/odom',
            'output_topic': '/kf_gins/path_iekf',
            'frame_id': 'map',
            'max_path_length': 20000,
            'min_distance': 0.05,
            'require_armed': True,
            'clear_on_arm_transition': False,
            'clear_on_reset_event': True,
            'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
            'startup_ignore_sec': 3.0,
            'max_abs_position_m': 10000.0,
            'max_abs_velocity_mps': 100.0,
            'max_jump_distance_m': 50.0,
            'mavros_state_stale_sec': 1.5,
            'armed_false_hold_sec': 6.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_iekf_path')),
    )

    # 真值轨迹发布器
    gt_path = Node(
        package='kf_gins_ros2_native',
        executable='odom_to_path.py',
        name='odom_to_path_ground_truth',
        output='screen',
        parameters=[{
            'odom_topic': '/kf_gins/odom',  # 可改为 Gazebo 真值
            'path_topic': '/kf_gins/path_ground_truth',
            'buffer_length': 1000
        }],
        condition=IfCondition(LaunchConfiguration('enable_gt_path')),
    )

    # ============ PX4 EKF2 (已通过 MAVROS 发布) ============
    # 注意: EKF2 状态通过 /mavros/local_position/pose 发布
    # 我们创建一个节点来重新发布为标准格式
    ekf2_relay = Node(
        package='kf_gins_ros2_native',
        executable='ekf2_state_relay.py',  # ⭐ 新脚本，下面会创建
        name='ekf2_state_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_mode': LaunchConfiguration('ekf2_input_mode'),
            'input_topic': LaunchConfiguration('mavros_local_pose_topic'),
            'vehicle_odometry_topic': LaunchConfiguration('px4_vehicle_odometry_topic'),
            'velocity_topic': LaunchConfiguration('mavros_local_velocity_topic'),
            'output_topic': '/ekf2/pose',
            'publish_pose': LaunchConfiguration('ekf2_relay_publish_pose'),
            # Preserve the source sample stamp so paired comparisons do not hide relay latency.
            'use_input_stamp': LaunchConfiguration('ekf2_use_input_stamp'),
            'use_covariance': False,
            'prefer_native_velocity': True,
            'max_native_velocity_age_sec': 0.5,
        }],
        condition=IfCondition(LaunchConfiguration('enable_ekf2_relay')),
    )

    # ============ KF-GINS IEKF ============
    kf_gins_config = LaunchConfiguration('kf_gins_param_file')
    
    kf_gins = Node(
        package='kf_gins_ros2_native',
        executable='kf_gins_node',
        name='kf_gins_node',
        output='screen',
        parameters=[
            kf_gins_config,
            {
                'imu_source': LaunchConfiguration('imu_source'),
                'imu_topic': LaunchConfiguration('mavros_imu_topic'),
                'px4_sensor_combined_topic': LaunchConfiguration('px4_sensor_combined_topic'),
                'px4_vehicle_imu_topic': LaunchConfiguration('px4_vehicle_imu_topic'),
                'px4_imu_qos_depth': LaunchConfiguration('px4_imu_qos_depth'),
                'gnss_source': LaunchConfiguration('gnss_source'),
                'gnss_topic': '/gps/fix',
                'px4_sensor_gps_topic': LaunchConfiguration('px4_sensor_gps_topic'),
                'px4_vehicle_global_position_topic': LaunchConfiguration('px4_vehicle_global_position_topic'),
                'config_file': LaunchConfiguration('kf_gins_core_config_file'),
                'force_zero_antlever': ParameterValue(LaunchConfiguration('force_zero_antlever'), value_type=bool),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_gnss_llh_for_pose': LaunchConfiguration('use_gnss_llh_for_pose'),
                'imu_input_is_flu': True,
                # Time base robustness: prefer values from YAML (kfgins_sim_fixed.yaml)
                # 【修复】仿真环境 GNSS 协方差自适应参数
                'use_sim_gnss_std': LaunchConfiguration('use_sim_gnss_std'),
                'sim_gnss_std_h_m': LaunchConfiguration('sim_gnss_std_h_m'),
                'sim_gnss_std_u_m': LaunchConfiguration('sim_gnss_std_u_m'),
                'gnss_position_lag_compensation_enable': LaunchConfiguration('gnss_position_lag_compensation_enable'),
                'gnss_position_lag_compensation_sec': LaunchConfiguration('gnss_position_lag_compensation_sec'),
                'gnss_position_lag_compensation_max_sec': LaunchConfiguration('gnss_position_lag_compensation_max_sec'),
                'gnss_position_lag_compensation_min_speed_mps': LaunchConfiguration('gnss_position_lag_compensation_min_speed_mps'),
                'gnss_update_debug_csv_path': LaunchConfiguration('gnss_update_debug_csv_path'),
                'gnss_nis_debug_csv_path': LaunchConfiguration('gnss_nis_debug_csv_path'),
                'gnss_nis_debug_max_rate_hz': ParameterValue(LaunchConfiguration('gnss_nis_debug_max_rate_hz'), value_type=float),
                'state_update_debug_csv_path': LaunchConfiguration('state_update_debug_csv_path'),
                'state_update_debug_max_rate_hz': ParameterValue(LaunchConfiguration('state_update_debug_max_rate_hz'), value_type=float),
                'dtrq_runtime_feature_debug_csv_path': LaunchConfiguration('dtrq_runtime_feature_debug_csv_path'),
                'dtrq_runtime_feature_debug_max_rate_hz': ParameterValue(LaunchConfiguration('dtrq_runtime_feature_debug_max_rate_hz'), value_type=float),
                'early_recovery_bias_feedback_debug_enable': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_debug_enable'), value_type=bool),
                'early_recovery_bias_feedback_apply_enable': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_apply_enable'), value_type=bool),
                'early_recovery_bias_feedback_history_sec': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_history_sec'), value_type=float),
                'early_recovery_bias_feedback_min_armed_time_sec': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_min_armed_time_sec'), value_type=float),
                'early_recovery_bias_feedback_max_armed_time_sec': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_max_armed_time_sec'), value_type=float),
                'early_recovery_bias_feedback_ba_z_mean_max_mps2': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_ba_z_mean_max_mps2'), value_type=float),
                'early_recovery_bias_feedback_residual_u_mean_max_m': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_residual_u_mean_max_m'), value_type=float),
                'early_recovery_bias_feedback_core_gnss_u_mean_min_m': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_core_gnss_u_mean_min_m'), value_type=float),
                'early_recovery_bias_feedback_dx_ba_z_sum_max_mps2': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_dx_ba_z_sum_max_mps2'), value_type=float),
                'early_recovery_bias_feedback_min_history_rows': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_min_history_rows'), value_type=int),
                'early_recovery_bias_feedback_negative_dx_scale': ParameterValue(LaunchConfiguration('early_recovery_bias_feedback_negative_dx_scale'), value_type=float),
                'horizontal_consistency_debug_csv_path': LaunchConfiguration('horizontal_consistency_debug_csv_path'),
                'heading_update_debug_csv_path': LaunchConfiguration('heading_update_debug_csv_path'),
                'state_publish_debug_csv_path': LaunchConfiguration('state_publish_debug_csv_path'),
                'shadow_restore_publish_enable': ParameterValue(LaunchConfiguration('shadow_restore_publish_enable'), value_type=bool),
                'shadow_restore_subscribe_enable': ParameterValue(LaunchConfiguration('shadow_restore_subscribe_enable'), value_type=bool),
                'shadow_restore_topic': LaunchConfiguration('shadow_restore_topic'),
                'shadow_restore_publish_after_core_sec': ParameterValue(LaunchConfiguration('shadow_restore_publish_after_core_sec'), value_type=float),
                'shadow_restore_publish_once': ParameterValue(LaunchConfiguration('shadow_restore_publish_once'), value_type=bool),
                'shadow_restore_publish_period_sec': ParameterValue(LaunchConfiguration('shadow_restore_publish_period_sec'), value_type=float),
                'shadow_restore_max_age_sec': ParameterValue(LaunchConfiguration('shadow_restore_max_age_sec'), value_type=float),
                'shadow_restore_covariance_inflation_factor': ParameterValue(LaunchConfiguration('shadow_restore_covariance_inflation_factor'), value_type=float),
                'shadow_restore_require_core_initialized': ParameterValue(LaunchConfiguration('shadow_restore_require_core_initialized'), value_type=bool),
                'shadow_restore_clear_path_on_apply': ParameterValue(LaunchConfiguration('shadow_restore_clear_path_on_apply'), value_type=bool),
                'shadow_restore_event_csv_path': LaunchConfiguration('shadow_restore_event_csv_path'),
                'shadow_supervisor_fsm_debug_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_debug_enable'), value_type=bool),
                'shadow_supervisor_fsm_reference_odom_topic': LaunchConfiguration('shadow_supervisor_fsm_reference_odom_topic'),
                'shadow_supervisor_fsm_debug_csv_path': LaunchConfiguration('shadow_supervisor_fsm_debug_csv_path'),
                'shadow_supervisor_fsm_events_csv_path': LaunchConfiguration('shadow_supervisor_fsm_events_csv_path'),
                'shadow_supervisor_fsm_debug_max_rate_hz': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_debug_max_rate_hz'), value_type=float),
                'shadow_supervisor_fsm_reference_odom_max_age_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_reference_odom_max_age_sec'), value_type=float),
                'shadow_supervisor_fsm_gamma_guard_enter': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_gamma_guard_enter'), value_type=float),
                'shadow_supervisor_fsm_gamma_shadow_max': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_gamma_shadow_max'), value_type=float),
                'shadow_supervisor_fsm_process_lambda_enter': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_process_lambda_enter'), value_type=float),
                'shadow_supervisor_fsm_process_score_min': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_process_score_min'), value_type=float),
                'shadow_supervisor_fsm_warmup_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_warmup_sec'), value_type=float),
                'shadow_supervisor_fsm_ready_confirm_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_ready_confirm_sec'), value_type=float),
                'shadow_supervisor_fsm_xy_delta_ready_max_m': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_xy_delta_ready_max_m'), value_type=float),
                'shadow_supervisor_fsm_z_delta_ready_max_m': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_z_delta_ready_max_m'), value_type=float),
                'shadow_supervisor_fsm_vel_delta_ready_max_mps': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_vel_delta_ready_max_mps'), value_type=float),
                'shadow_supervisor_fsm_yaw_delta_ready_max_deg': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_yaw_delta_ready_max_deg'), value_type=float),
                'shadow_supervisor_fsm_allow_mixed_trigger': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_allow_mixed_trigger'), value_type=bool),
                'shadow_supervisor_fsm_observation_score_guard_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_observation_score_guard_enable'), value_type=bool),
                'shadow_supervisor_fsm_observation_score_guard_enter': ParameterValue(LaunchConfiguration('shadow_supervisor_fsm_observation_score_guard_enter'), value_type=float),
                'shadow_supervisor_perf_proxy_publish_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_publish_enable'), value_type=bool),
                'shadow_supervisor_perf_proxy_subscribe_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_subscribe_enable'), value_type=bool),
                'shadow_supervisor_perf_proxy_topic': LaunchConfiguration('shadow_supervisor_perf_proxy_topic'),
                'shadow_supervisor_perf_proxy_max_age_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_max_age_sec'), value_type=float),
                'shadow_supervisor_perf_proxy_short_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_short_window_sec'), value_type=float),
                'shadow_supervisor_perf_proxy_long_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_long_window_sec'), value_type=float),
                'shadow_supervisor_perf_proxy_residual_gap_soft_max_m': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_residual_gap_soft_max_m'), value_type=float),
                'shadow_supervisor_perf_proxy_nis_gap_permission_max': ParameterValue(LaunchConfiguration('shadow_supervisor_perf_proxy_nis_gap_permission_max'), value_type=float),
                'shadow_supervisor_predictive_score_publish_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_publish_enable'), value_type=bool),
                'shadow_supervisor_predictive_score_subscribe_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_subscribe_enable'), value_type=bool),
                'shadow_supervisor_predictive_score_debug_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_debug_enable'), value_type=bool),
                'shadow_supervisor_predictive_score_topic': LaunchConfiguration('shadow_supervisor_predictive_score_topic'),
                'shadow_supervisor_predictive_score_debug_csv_path': LaunchConfiguration('shadow_supervisor_predictive_score_debug_csv_path'),
                'shadow_supervisor_predictive_score_source_id': LaunchConfiguration('shadow_supervisor_predictive_score_source_id'),
                'shadow_supervisor_predictive_score_max_age_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_max_age_sec'), value_type=float),
                'shadow_supervisor_predictive_score_stamp_tolerance_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_stamp_tolerance_sec'), value_type=float),
                'shadow_supervisor_predictive_score_short_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_short_window_sec'), value_type=float),
                'shadow_supervisor_predictive_score_long_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_long_window_sec'), value_type=float),
                'shadow_supervisor_predictive_score_eval_std_h_m': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_eval_std_h_m'), value_type=float),
                'shadow_supervisor_predictive_score_eval_std_u_m': ParameterValue(LaunchConfiguration('shadow_supervisor_predictive_score_eval_std_u_m'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_publish_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_publish_enable'), value_type=bool),
                'shadow_supervisor_velocity_predictive_score_subscribe_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_subscribe_enable'), value_type=bool),
                'shadow_supervisor_velocity_predictive_score_debug_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_debug_enable'), value_type=bool),
                'shadow_supervisor_velocity_predictive_score_topic': LaunchConfiguration('shadow_supervisor_velocity_predictive_score_topic'),
                'shadow_supervisor_velocity_predictive_score_debug_csv_path': LaunchConfiguration('shadow_supervisor_velocity_predictive_score_debug_csv_path'),
                'shadow_supervisor_velocity_predictive_score_source_id': LaunchConfiguration('shadow_supervisor_velocity_predictive_score_source_id'),
                'shadow_supervisor_velocity_predictive_score_max_age_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_max_age_sec'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_short_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_short_window_sec'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_long_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_long_window_sec'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_eval_std_h_mps': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_eval_std_h_mps'), value_type=float),
                'shadow_supervisor_velocity_predictive_score_eval_std_u_mps': ParameterValue(LaunchConfiguration('shadow_supervisor_velocity_predictive_score_eval_std_u_mps'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_publish_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_publish_enable'), value_type=bool),
                'shadow_supervisor_kinematic_predictive_score_subscribe_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_subscribe_enable'), value_type=bool),
                'shadow_supervisor_kinematic_predictive_score_debug_enable': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_debug_enable'), value_type=bool),
                'shadow_supervisor_kinematic_predictive_score_topic': LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_topic'),
                'shadow_supervisor_kinematic_predictive_score_debug_csv_path': LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_debug_csv_path'),
                'shadow_supervisor_kinematic_predictive_score_source_id': LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_source_id'),
                'shadow_supervisor_kinematic_predictive_score_max_age_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_max_age_sec'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_short_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_short_window_sec'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_long_window_sec': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_long_window_sec'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_eval_std_h_m': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_eval_std_h_m'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_eval_std_u_m': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_eval_std_u_m'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_eval_std_h_mps': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_eval_std_h_mps'), value_type=float),
                'shadow_supervisor_kinematic_predictive_score_eval_std_u_mps': ParameterValue(LaunchConfiguration('shadow_supervisor_kinematic_predictive_score_eval_std_u_mps'), value_type=float),
                'segment_timing_gate_debug_csv_path': LaunchConfiguration('segment_timing_gate_debug_csv_path'),
                'segment_timing_gate_segment_sec': ParameterValue(LaunchConfiguration('segment_timing_gate_segment_sec'), value_type=float),
                'segment_timing_gate_lag_threshold_sec': ParameterValue(LaunchConfiguration('segment_timing_gate_lag_threshold_sec'), value_type=float),
                'segment_timing_gate_gnss_source_age_max_sec': ParameterValue(LaunchConfiguration('segment_timing_gate_gnss_source_age_max_sec'), value_type=float),
                'segment_timing_gate_core_gnss_along_min_enable': ParameterValue(LaunchConfiguration('segment_timing_gate_core_gnss_along_min_enable'), value_type=bool),
                'segment_timing_gate_core_gnss_along_min_threshold_m': ParameterValue(LaunchConfiguration('segment_timing_gate_core_gnss_along_min_threshold_m'), value_type=float),
                'segment_timing_gate_projection_alpha': ParameterValue(LaunchConfiguration('segment_timing_gate_projection_alpha'), value_type=float),
                'segment_timing_gate_projection_enable': ParameterValue(LaunchConfiguration('segment_timing_gate_projection_enable'), value_type=bool),
                'segment_timing_gate_projection_apply_mission': ParameterValue(LaunchConfiguration('segment_timing_gate_projection_apply_mission'), value_type=bool),
                'segment_timing_gate_projection_apply_rtl': ParameterValue(LaunchConfiguration('segment_timing_gate_projection_apply_rtl'), value_type=bool),
                'segment_timing_gate_projection_apply_other': ParameterValue(LaunchConfiguration('segment_timing_gate_projection_apply_other'), value_type=bool),
                'publish_state_after_gnss_update': ParameterValue(LaunchConfiguration('publish_state_after_gnss_update'), value_type=bool),
                'publish_px4_sphere_projection': ParameterValue(LaunchConfiguration('publish_px4_sphere_projection'), value_type=bool),
                'accbias_z_history_projection_enable': ParameterValue(LaunchConfiguration('accbias_z_history_projection_enable'), value_type=bool),
                'accbias_z_history_projection_apply_mission': ParameterValue(LaunchConfiguration('accbias_z_history_projection_apply_mission'), value_type=bool),
                'accbias_z_history_projection_apply_rtl': ParameterValue(LaunchConfiguration('accbias_z_history_projection_apply_rtl'), value_type=bool),
                'accbias_z_history_projection_apply_other': ParameterValue(LaunchConfiguration('accbias_z_history_projection_apply_other'), value_type=bool),
                'accbias_z_history_projection_alpha': ParameterValue(LaunchConfiguration('accbias_z_history_projection_alpha'), value_type=float),
                'accbias_z_history_projection_deep_threshold_mps2': ParameterValue(LaunchConfiguration('accbias_z_history_projection_deep_threshold_mps2'), value_type=float),
                'accbias_z_history_projection_frac_threshold': ParameterValue(LaunchConfiguration('accbias_z_history_projection_frac_threshold'), value_type=float),
                'accbias_z_history_projection_history_start_sec': ParameterValue(LaunchConfiguration('accbias_z_history_projection_history_start_sec'), value_type=float),
                'publish_stamp_mode': LaunchConfiguration('publish_stamp_mode'),
                'publish_core_stamp_max_future_sec': ParameterValue(LaunchConfiguration('publish_core_stamp_max_future_sec'), value_type=float),
                'publish_core_stamp_max_past_sec': ParameterValue(LaunchConfiguration('publish_core_stamp_max_past_sec'), value_type=float),
                'publish_core_stamp_offset_bias_sec': ParameterValue(LaunchConfiguration('publish_core_stamp_offset_bias_sec'), value_type=float),
                'raw_odom_decimation': ParameterValue(LaunchConfiguration('raw_odom_decimation'), value_type=int),
                'path_publish_rate_hz': ParameterValue(LaunchConfiguration('path_publish_rate_hz'), value_type=float),
                'pose_decimation': ParameterValue(LaunchConfiguration('pose_decimation'), value_type=int),
                'max_path_points': ParameterValue(LaunchConfiguration('max_path_points'), value_type=int),
                'core_processing_enable': ParameterValue(LaunchConfiguration('core_processing_enable'), value_type=bool),
                'core_imu_decimation': ParameterValue(LaunchConfiguration('core_imu_decimation'), value_type=int),
                'core_max_imu_rate_hz': ParameterValue(LaunchConfiguration('core_max_imu_rate_hz'), value_type=float),
                'armed_cruise_native_gnss_vel_override_enable': LaunchConfiguration('armed_cruise_native_gnss_vel_override_enable'),
                'armed_cruise_gnss_pos_override_enable': LaunchConfiguration('armed_cruise_gnss_pos_override_enable'),
                'armed_cruise_gnss_pos_override_apply_mission': LaunchConfiguration('armed_cruise_gnss_pos_override_apply_mission'),
                'armed_cruise_gnss_pos_override_apply_rtl': LaunchConfiguration('armed_cruise_gnss_pos_override_apply_rtl'),
                'armed_cruise_gnss_pos_override_apply_other': LaunchConfiguration('armed_cruise_gnss_pos_override_apply_other'),
                'armed_cruise_gnss_pos_std_h_m': LaunchConfiguration('armed_cruise_gnss_pos_std_h_m'),
                'armed_cruise_gnss_pos_std_u_m': LaunchConfiguration('armed_cruise_gnss_pos_std_u_m'),
                'armed_cruise_gnss_pos_residual_boost_enable': LaunchConfiguration('armed_cruise_gnss_pos_residual_boost_enable'),
                'armed_cruise_gnss_pos_residual_boost_threshold_m': LaunchConfiguration('armed_cruise_gnss_pos_residual_boost_threshold_m'),
                'armed_cruise_gnss_pos_residual_boost_hold_sec': LaunchConfiguration('armed_cruise_gnss_pos_residual_boost_hold_sec'),
                'gnss_position_response_boost_enable': ParameterValue(LaunchConfiguration('gnss_position_response_boost_enable'), value_type=bool),
                'gnss_position_response_boost_apply_mission': ParameterValue(LaunchConfiguration('gnss_position_response_boost_apply_mission'), value_type=bool),
                'gnss_position_response_boost_apply_rtl': ParameterValue(LaunchConfiguration('gnss_position_response_boost_apply_rtl'), value_type=bool),
                'gnss_position_response_boost_residual_start_h_m': ParameterValue(LaunchConfiguration('gnss_position_response_boost_residual_start_h_m'), value_type=float),
                'gnss_position_response_boost_residual_full_h_m': ParameterValue(LaunchConfiguration('gnss_position_response_boost_residual_full_h_m'), value_type=float),
                'gnss_position_response_boost_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('gnss_position_response_boost_min_horizontal_speed_mps'), value_type=float),
                'gnss_position_response_boost_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('gnss_position_response_boost_max_abs_vertical_speed_mps'), value_type=float),
                'gnss_position_response_boost_persistence_updates': ParameterValue(LaunchConfiguration('gnss_position_response_boost_persistence_updates'), value_type=int),
                'gnss_position_response_boost_std_min_h_m': ParameterValue(LaunchConfiguration('gnss_position_response_boost_std_min_h_m'), value_type=float),
                'gnss_position_response_boost_std_max_h_m': ParameterValue(LaunchConfiguration('gnss_position_response_boost_std_max_h_m'), value_type=float),
                'gnss_position_gain_response_enable': ParameterValue(LaunchConfiguration('gnss_position_gain_response_enable'), value_type=bool),
                'gnss_position_gain_response_apply_mission': ParameterValue(LaunchConfiguration('gnss_position_gain_response_apply_mission'), value_type=bool),
                'gnss_position_gain_response_apply_rtl': ParameterValue(LaunchConfiguration('gnss_position_gain_response_apply_rtl'), value_type=bool),
                'gnss_position_gain_response_require_armed_cruise': ParameterValue(LaunchConfiguration('gnss_position_gain_response_require_armed_cruise'), value_type=bool),
                'gnss_position_gain_response_block_turning': ParameterValue(LaunchConfiguration('gnss_position_gain_response_block_turning'), value_type=bool),
                'gnss_position_gain_response_block_post_turn': ParameterValue(LaunchConfiguration('gnss_position_gain_response_block_post_turn'), value_type=bool),
                'gnss_position_gain_response_residual_start_h_m': ParameterValue(LaunchConfiguration('gnss_position_gain_response_residual_start_h_m'), value_type=float),
                'gnss_position_gain_response_residual_full_h_m': ParameterValue(LaunchConfiguration('gnss_position_gain_response_residual_full_h_m'), value_type=float),
                'gnss_position_gain_response_hnis_start': ParameterValue(LaunchConfiguration('gnss_position_gain_response_hnis_start'), value_type=float),
                'gnss_position_gain_response_hnis_full': ParameterValue(LaunchConfiguration('gnss_position_gain_response_hnis_full'), value_type=float),
                'gnss_position_gain_response_prev_gain_low': ParameterValue(LaunchConfiguration('gnss_position_gain_response_prev_gain_low'), value_type=float),
                'gnss_position_gain_response_prev_gain_high': ParameterValue(LaunchConfiguration('gnss_position_gain_response_prev_gain_high'), value_type=float),
                'gnss_position_gain_response_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('gnss_position_gain_response_min_horizontal_speed_mps'), value_type=float),
                'gnss_position_gain_response_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('gnss_position_gain_response_max_abs_vertical_speed_mps'), value_type=float),
                'gnss_position_gain_response_persistence_updates': ParameterValue(LaunchConfiguration('gnss_position_gain_response_persistence_updates'), value_type=int),
                'gnss_position_gain_response_std_min_h_m': ParameterValue(LaunchConfiguration('gnss_position_gain_response_std_min_h_m'), value_type=float),
                'gnss_position_gain_response_std_max_h_m': ParameterValue(LaunchConfiguration('gnss_position_gain_response_std_max_h_m'), value_type=float),
                'gnss_velocity_outward_damping_enable': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_enable'), value_type=bool),
                'gnss_velocity_outward_damping_apply_mission': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_apply_mission'), value_type=bool),
                'gnss_velocity_outward_damping_apply_rtl': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_apply_rtl'), value_type=bool),
                'gnss_velocity_outward_damping_min_core_gnss_diff_h_m': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_min_core_gnss_diff_h_m'), value_type=float),
                'gnss_velocity_outward_damping_radial_start_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_radial_start_mps'), value_type=float),
                'gnss_velocity_outward_damping_radial_full_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_radial_full_mps'), value_type=float),
                'gnss_velocity_outward_damping_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_min_horizontal_speed_mps'), value_type=float),
                'gnss_velocity_outward_damping_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_max_abs_vertical_speed_mps'), value_type=float),
                'gnss_velocity_outward_damping_persistence_updates': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_persistence_updates'), value_type=int),
                'gnss_velocity_outward_damping_hold_updates': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_hold_updates'), value_type=int),
                'gnss_velocity_outward_damping_std_min_h_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_std_min_h_mps'), value_type=float),
                'gnss_velocity_outward_damping_std_max_h_mps': ParameterValue(LaunchConfiguration('gnss_velocity_outward_damping_std_max_h_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_enable': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_enable'), value_type=bool),
                'turn_postturn_native_velocity_deweight_apply_mission': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_apply_mission'), value_type=bool),
                'turn_postturn_native_velocity_deweight_apply_rtl': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_apply_rtl'), value_type=bool),
                'turn_postturn_native_velocity_deweight_apply_turning': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_apply_turning'), value_type=bool),
                'turn_postturn_native_velocity_deweight_apply_post_turn': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_apply_post_turn'), value_type=bool),
                'turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m'), value_type=float),
                'turn_postturn_native_velocity_deweight_radial_abs_start_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_radial_abs_start_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_radial_abs_full_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_radial_abs_full_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_core_residual_start_h_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_core_residual_start_h_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_core_residual_full_h_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_core_residual_full_h_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_min_horizontal_speed_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_persistence_updates': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_persistence_updates'), value_type=int),
                'turn_postturn_native_velocity_deweight_std_min_h_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_std_min_h_mps'), value_type=float),
                'turn_postturn_native_velocity_deweight_std_max_h_mps': ParameterValue(LaunchConfiguration('turn_postturn_native_velocity_deweight_std_max_h_mps'), value_type=float),
                'phase_error_memory_debug_enable': ParameterValue(LaunchConfiguration('phase_error_memory_debug_enable'), value_type=bool),
                'phase_error_memory_debug_residual_threshold_h_m': ParameterValue(LaunchConfiguration('phase_error_memory_debug_residual_threshold_h_m'), value_type=float),
                'phase_error_memory_debug_dx_over_residual_threshold': ParameterValue(LaunchConfiguration('phase_error_memory_debug_dx_over_residual_threshold'), value_type=float),
                'phase_error_memory_debug_recent_turnpost_hold_sec': ParameterValue(LaunchConfiguration('phase_error_memory_debug_recent_turnpost_hold_sec'), value_type=float),
                'adaptive_gnss_pos_weight_enable': LaunchConfiguration('adaptive_gnss_pos_weight_enable'),
                'adaptive_gnss_pos_weight_apply_mission': LaunchConfiguration('adaptive_gnss_pos_weight_apply_mission'),
                'adaptive_gnss_pos_weight_apply_rtl': LaunchConfiguration('adaptive_gnss_pos_weight_apply_rtl'),
                'adaptive_gnss_pos_weight_trigger_source': LaunchConfiguration('adaptive_gnss_pos_weight_trigger_source'),
                'adaptive_gnss_pos_weight_floor_min_h_m': LaunchConfiguration('adaptive_gnss_pos_weight_floor_min_h_m'),
                'adaptive_gnss_pos_weight_floor_nominal_h_m': LaunchConfiguration('adaptive_gnss_pos_weight_floor_nominal_h_m'),
                'adaptive_gnss_pos_weight_floor_max_h_m': LaunchConfiguration('adaptive_gnss_pos_weight_floor_max_h_m'),
                'adaptive_gnss_pos_weight_residual_start_h_m': LaunchConfiguration('adaptive_gnss_pos_weight_residual_start_h_m'),
                'adaptive_gnss_pos_weight_residual_full_h_m': LaunchConfiguration('adaptive_gnss_pos_weight_residual_full_h_m'),
                'adaptive_gnss_pos_weight_nis_start_h_2d': LaunchConfiguration('adaptive_gnss_pos_weight_nis_start_h_2d'),
                'adaptive_gnss_pos_weight_nis_full_h_2d': LaunchConfiguration('adaptive_gnss_pos_weight_nis_full_h_2d'),
                'adaptive_gnss_pos_weight_nis_max_age_sec': LaunchConfiguration('adaptive_gnss_pos_weight_nis_max_age_sec'),
                'adaptive_gnss_pos_weight_persistence_updates': ParameterValue(LaunchConfiguration('adaptive_gnss_pos_weight_persistence_updates'), value_type=int),
                'adaptive_gnss_pos_weight_attack_sec': LaunchConfiguration('adaptive_gnss_pos_weight_attack_sec'),
                'adaptive_gnss_pos_weight_decay_sec': LaunchConfiguration('adaptive_gnss_pos_weight_decay_sec'),
                'adaptive_gnss_pos_weight_armed_cruise_gain': LaunchConfiguration('adaptive_gnss_pos_weight_armed_cruise_gain'),
                'adaptive_gnss_pos_weight_turn_gain': LaunchConfiguration('adaptive_gnss_pos_weight_turn_gain'),
                'adaptive_gnss_pos_weight_post_turn_gain': LaunchConfiguration('adaptive_gnss_pos_weight_post_turn_gain'),
                'horizontal_consistency_supervisor_enable': ParameterValue(LaunchConfiguration('horizontal_consistency_supervisor_enable'), value_type=bool),
                'horizontal_consistency_apply_mission': ParameterValue(LaunchConfiguration('horizontal_consistency_apply_mission'), value_type=bool),
                'horizontal_consistency_apply_rtl': ParameterValue(LaunchConfiguration('horizontal_consistency_apply_rtl'), value_type=bool),
                'horizontal_consistency_nis_start_h_2d': ParameterValue(LaunchConfiguration('horizontal_consistency_nis_start_h_2d'), value_type=float),
                'horizontal_consistency_nis_full_h_2d': ParameterValue(LaunchConfiguration('horizontal_consistency_nis_full_h_2d'), value_type=float),
                'horizontal_consistency_residual_start_h_m': ParameterValue(LaunchConfiguration('horizontal_consistency_residual_start_h_m'), value_type=float),
                'horizontal_consistency_residual_full_h_m': ParameterValue(LaunchConfiguration('horizontal_consistency_residual_full_h_m'), value_type=float),
                'horizontal_consistency_core_gnss_start_h_m': ParameterValue(LaunchConfiguration('horizontal_consistency_core_gnss_start_h_m'), value_type=float),
                'horizontal_consistency_core_gnss_full_h_m': ParameterValue(LaunchConfiguration('horizontal_consistency_core_gnss_full_h_m'), value_type=float),
                'horizontal_consistency_heading_residual_start_deg': ParameterValue(LaunchConfiguration('horizontal_consistency_heading_residual_start_deg'), value_type=float),
                'horizontal_consistency_heading_residual_full_deg': ParameterValue(LaunchConfiguration('horizontal_consistency_heading_residual_full_deg'), value_type=float),
                'horizontal_consistency_score_trigger': ParameterValue(LaunchConfiguration('horizontal_consistency_score_trigger'), value_type=float),
                'horizontal_consistency_persistence_updates': ParameterValue(LaunchConfiguration('horizontal_consistency_persistence_updates'), value_type=int),
                'horizontal_consistency_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('horizontal_consistency_min_horizontal_speed_mps'), value_type=float),
                'horizontal_consistency_max_vertical_speed_mps': ParameterValue(LaunchConfiguration('horizontal_consistency_max_vertical_speed_mps'), value_type=float),
                'mission_cov_hygiene_enable': ParameterValue(LaunchConfiguration('mission_cov_hygiene_enable'), value_type=bool),
                'mission_cov_hygiene_apply_mission': ParameterValue(LaunchConfiguration('mission_cov_hygiene_apply_mission'), value_type=bool),
                'mission_cov_hygiene_apply_rtl': ParameterValue(LaunchConfiguration('mission_cov_hygiene_apply_rtl'), value_type=bool),
                'mission_cov_hygiene_hnis_start': ParameterValue(LaunchConfiguration('mission_cov_hygiene_hnis_start'), value_type=float),
                'mission_cov_hygiene_hnis_full': ParameterValue(LaunchConfiguration('mission_cov_hygiene_hnis_full'), value_type=float),
                'mission_cov_hygiene_resid_start_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_resid_start_h_m'), value_type=float),
                'mission_cov_hygiene_resid_full_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_resid_full_h_m'), value_type=float),
                'mission_cov_hygiene_pos_std_tight_lo_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_pos_std_tight_lo_h_m'), value_type=float),
                'mission_cov_hygiene_pos_std_tight_hi_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_pos_std_tight_hi_h_m'), value_type=float),
                'mission_cov_hygiene_dx_ratio_low': ParameterValue(LaunchConfiguration('mission_cov_hygiene_dx_ratio_low'), value_type=float),
                'mission_cov_hygiene_dx_ratio_high': ParameterValue(LaunchConfiguration('mission_cov_hygiene_dx_ratio_high'), value_type=float),
                'mission_cov_hygiene_persistence_updates': ParameterValue(LaunchConfiguration('mission_cov_hygiene_persistence_updates'), value_type=int),
                'mission_cov_hygiene_attack_sec': ParameterValue(LaunchConfiguration('mission_cov_hygiene_attack_sec'), value_type=float),
                'mission_cov_hygiene_decay_sec': ParameterValue(LaunchConfiguration('mission_cov_hygiene_decay_sec'), value_type=float),
                'mission_cov_hygiene_floor_min_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_floor_min_h_m'), value_type=float),
                'mission_cov_hygiene_floor_nominal_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_floor_nominal_h_m'), value_type=float),
                'mission_cov_hygiene_floor_max_h_m': ParameterValue(LaunchConfiguration('mission_cov_hygiene_floor_max_h_m'), value_type=float),
                'mission_cov_hygiene_offdiag_corr_limit': ParameterValue(LaunchConfiguration('mission_cov_hygiene_offdiag_corr_limit'), value_type=float),
                'turn_rate_propagation_noise_probe_enable': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_enable'), value_type=bool),
                'turn_rate_propagation_noise_probe_apply_mission': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_apply_mission'), value_type=bool),
                'turn_rate_propagation_noise_probe_apply_rtl': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_apply_rtl'), value_type=bool),
                'turn_rate_propagation_noise_probe_gyro_start_deg_s': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_gyro_start_deg_s'), value_type=float),
                'turn_rate_propagation_noise_probe_gyro_full_deg_s': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_gyro_full_deg_s'), value_type=float),
                'turn_rate_propagation_noise_probe_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_min_horizontal_speed_mps'), value_type=float),
                'turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps'), value_type=float),
                'turn_rate_propagation_noise_probe_arw_q_scale_max': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_arw_q_scale_max'), value_type=float),
                'turn_rate_propagation_noise_probe_vrw_q_scale_max': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_vrw_q_scale_max'), value_type=float),
                'turn_rate_propagation_noise_probe_gyrbias_q_scale_max': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_gyrbias_q_scale_max'), value_type=float),
                'turn_rate_propagation_noise_probe_accbias_q_scale_max': ParameterValue(LaunchConfiguration('turn_rate_propagation_noise_probe_accbias_q_scale_max'), value_type=float),
                'accbias_z_propagation_probe_enable': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_enable'), value_type=bool),
                'accbias_z_propagation_probe_apply_noise_scale': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_apply_noise_scale'), value_type=bool),
                'accbias_z_propagation_probe_apply_mission': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_apply_mission'), value_type=bool),
                'accbias_z_propagation_probe_apply_rtl': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_apply_rtl'), value_type=bool),
                'accbias_z_propagation_probe_trigger_mps2': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_trigger_mps2'), value_type=float),
                'accbias_z_propagation_probe_full_mps2': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_full_mps2'), value_type=float),
                'accbias_z_propagation_probe_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_min_horizontal_speed_mps'), value_type=float),
                'accbias_z_propagation_probe_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_max_abs_vertical_speed_mps'), value_type=float),
                'accbias_z_propagation_probe_arw_q_scale_max': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_arw_q_scale_max'), value_type=float),
                'accbias_z_propagation_probe_vrw_q_scale_max': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_vrw_q_scale_max'), value_type=float),
                'accbias_z_propagation_probe_gyrbias_q_scale_max': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_gyrbias_q_scale_max'), value_type=float),
                'accbias_z_propagation_probe_accbias_q_scale_max': ParameterValue(LaunchConfiguration('accbias_z_propagation_probe_accbias_q_scale_max'), value_type=float),
                'motion_gnss_pos_weight_enable': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_enable'), value_type=bool),
                'motion_gnss_pos_weight_apply_mission': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_apply_mission'), value_type=bool),
                'motion_gnss_pos_weight_apply_rtl': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_apply_rtl'), value_type=bool),
                'motion_gnss_pos_weight_hspeed_start_mps': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_hspeed_start_mps'), value_type=float),
                'motion_gnss_pos_weight_hspeed_full_mps': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_hspeed_full_mps'), value_type=float),
                'motion_gnss_pos_weight_max_abs_vspeed_mps': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_max_abs_vspeed_mps'), value_type=float),
                'motion_gnss_pos_weight_std_min_h_m': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_std_min_h_m'), value_type=float),
                'motion_gnss_pos_weight_std_max_h_m': ParameterValue(LaunchConfiguration('motion_gnss_pos_weight_std_max_h_m'), value_type=float),
                'gnss_pos_recovery_weight_enable': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_enable'), value_type=bool),
                'gnss_pos_recovery_weight_apply_mission': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_apply_mission'), value_type=bool),
                'gnss_pos_recovery_weight_apply_rtl': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_apply_rtl'), value_type=bool),
                'gnss_pos_recovery_weight_residual_start_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_residual_start_h_m'), value_type=float),
                'gnss_pos_recovery_weight_residual_full_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_residual_full_h_m'), value_type=float),
                'gnss_pos_recovery_weight_core_start_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_core_start_h_m'), value_type=float),
                'gnss_pos_recovery_weight_core_full_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_core_full_h_m'), value_type=float),
                'gnss_pos_recovery_weight_core_score_gain': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_core_score_gain'), value_type=float),
                'gnss_pos_recovery_weight_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_min_horizontal_speed_mps'), value_type=float),
                'gnss_pos_recovery_weight_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_max_abs_vertical_speed_mps'), value_type=float),
                'gnss_pos_recovery_weight_persistence_updates': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_persistence_updates'), value_type=int),
                'gnss_pos_recovery_weight_hold_sec': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_hold_sec'), value_type=float),
                'gnss_pos_recovery_weight_attack_sec': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_attack_sec'), value_type=float),
                'gnss_pos_recovery_weight_decay_sec': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_decay_sec'), value_type=float),
                'gnss_pos_recovery_weight_std_min_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_std_min_h_m'), value_type=float),
                'gnss_pos_recovery_weight_std_max_h_m': ParameterValue(LaunchConfiguration('gnss_pos_recovery_weight_std_max_h_m'), value_type=float),
                'context_gnss_pos_floor_enable': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_enable'), value_type=bool),
                'context_gnss_pos_floor_apply_mission': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_apply_mission'), value_type=bool),
                'context_gnss_pos_floor_apply_rtl': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_apply_rtl'), value_type=bool),
                'context_gnss_pos_floor_mission_base_h_m': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_mission_base_h_m'), value_type=float),
                'context_gnss_pos_floor_turn_post_h_m': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_turn_post_h_m'), value_type=float),
                'context_gnss_pos_floor_armed_cruise_h_m': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_armed_cruise_h_m'), value_type=float),
                'context_gnss_pos_floor_rtl_h_m': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_rtl_h_m'), value_type=float),
                'context_gnss_pos_floor_attack_sec': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_attack_sec'), value_type=float),
                'context_gnss_pos_floor_decay_sec': ParameterValue(LaunchConfiguration('context_gnss_pos_floor_decay_sec'), value_type=float),
                'heading_cruise_micro_track_enable': LaunchConfiguration('heading_cruise_micro_track_enable'),
                'heading_cruise_micro_track_std_deg': LaunchConfiguration('heading_cruise_micro_track_std_deg'),
                'heading_cruise_micro_track_min_residual_deg': LaunchConfiguration('heading_cruise_micro_track_min_residual_deg'),
                'heading_cruise_micro_track_max_residual_deg': LaunchConfiguration('heading_cruise_micro_track_max_residual_deg'),
                'heading_cruise_micro_track_min_horizontal_speed_mps': LaunchConfiguration('heading_cruise_micro_track_min_horizontal_speed_mps'),
                'heading_cruise_micro_track_max_vertical_speed_mps': LaunchConfiguration('heading_cruise_micro_track_max_vertical_speed_mps'),
                'heading_cruise_micro_track_max_gyro_deg_s': LaunchConfiguration('heading_cruise_micro_track_max_gyro_deg_s'),
                'heading_cruise_micro_track_max_source_yaw_rate_deg_s': LaunchConfiguration('heading_cruise_micro_track_max_source_yaw_rate_deg_s'),
                'heading_yaw_gain_hygiene_enable': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_enable'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_mission': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_mission'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_rtl': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_rtl'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_armed_cruise': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_armed_cruise'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_post_turn': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_post_turn'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_turn': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_turn'), value_type=bool),
                'heading_yaw_gain_hygiene_apply_update': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_apply_update'), value_type=bool),
                'heading_yaw_gain_hygiene_yaw_std_floor_deg': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_yaw_std_floor_deg'), value_type=float),
                'heading_yaw_gain_hygiene_min_residual_deg': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_min_residual_deg'), value_type=float),
                'heading_yaw_gain_hygiene_max_residual_deg': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_max_residual_deg'), value_type=float),
                'heading_yaw_gain_hygiene_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_min_horizontal_speed_mps'), value_type=float),
                'heading_yaw_gain_hygiene_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_max_abs_vertical_speed_mps'), value_type=float),
                'heading_yaw_gain_hygiene_max_gyro_deg_s': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_max_gyro_deg_s'), value_type=float),
                'heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s'), value_type=float),
                'heading_yaw_gain_hygiene_max_rate_hz': ParameterValue(LaunchConfiguration('heading_yaw_gain_hygiene_max_rate_hz'), value_type=float),
                'armed_cruise_native_gnss_vel_min_horizontal_speed_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_min_horizontal_speed_mps'),
                'armed_cruise_native_gnss_vel_std_h_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_std_h_mps'),
                'armed_cruise_native_gnss_vel_std_u_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_std_u_mps'),
                'armed_cruise_native_gnss_vel_residual_boost_enable': LaunchConfiguration('armed_cruise_native_gnss_vel_residual_boost_enable'),
                'armed_cruise_native_gnss_vel_residual_boost_threshold_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_residual_boost_threshold_mps'),
                'armed_cruise_native_gnss_vel_residual_boost_hold_sec': LaunchConfiguration('armed_cruise_native_gnss_vel_residual_boost_hold_sec'),
                'armed_cruise_native_gnss_vel_residual_boost_std_h_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_residual_boost_std_h_mps'),
                'armed_cruise_native_gnss_vel_residual_boost_std_u_mps': LaunchConfiguration('armed_cruise_native_gnss_vel_residual_boost_std_u_mps'),
                'native_gnss_velocity_outlier_guard_enable': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_enable'), value_type=bool),
                'native_gnss_velocity_outlier_guard_apply_mission': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_apply_mission'), value_type=bool),
                'native_gnss_velocity_outlier_guard_apply_rtl': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_apply_rtl'), value_type=bool),
                'native_gnss_velocity_outlier_guard_speed_mismatch_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_speed_mismatch_mps'), value_type=float),
                'native_gnss_velocity_outlier_guard_core_residual_h_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_core_residual_h_mps'), value_type=float),
                'native_gnss_velocity_outlier_guard_min_horizontal_speed_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_min_horizontal_speed_mps'), value_type=float),
                'native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps'), value_type=float),
                'native_gnss_velocity_outlier_guard_apply_turning_context': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_apply_turning_context'), value_type=bool),
                'native_gnss_velocity_outlier_guard_action': LaunchConfiguration('native_gnss_velocity_outlier_guard_action'),
                'native_gnss_velocity_outlier_guard_reweight_std_h_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_reweight_std_h_mps'), value_type=float),
                'native_gnss_velocity_outlier_guard_reweight_std_u_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_outlier_guard_reweight_std_u_mps'), value_type=float),
                'native_gnss_velocity_low_speed_turn_source_guard_enable': ParameterValue(LaunchConfiguration('native_gnss_velocity_low_speed_turn_source_guard_enable'), value_type=bool),
                'native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps'), value_type=float),
                'native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s': ParameterValue(LaunchConfiguration('native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s'), value_type=float),
                'native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps': ParameterValue(LaunchConfiguration('native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps'), value_type=float),
                'armed_cruise_vertical_cov_reopen_enable': LaunchConfiguration('armed_cruise_vertical_cov_reopen_enable'),
                'armed_cruise_vertical_cov_reopen_threshold_m': LaunchConfiguration('armed_cruise_vertical_cov_reopen_threshold_m'),
                'armed_cruise_vertical_cov_reopen_hold_sec': LaunchConfiguration('armed_cruise_vertical_cov_reopen_hold_sec'),
                'armed_cruise_vertical_cov_reopen_pos_std_m': LaunchConfiguration('armed_cruise_vertical_cov_reopen_pos_std_m'),
                'armed_cruise_vertical_cov_reopen_vel_std_mps': LaunchConfiguration('armed_cruise_vertical_cov_reopen_vel_std_mps'),
                'armed_cruise_vertical_cov_reopen_accbias_std_z_mps2': LaunchConfiguration('armed_cruise_vertical_cov_reopen_accbias_std_z_mps2'),
                'post_flight_vertical_cov_reopen_enable': LaunchConfiguration('post_flight_vertical_cov_reopen_enable'),
                'post_flight_vertical_cov_reopen_threshold_m': LaunchConfiguration('post_flight_vertical_cov_reopen_threshold_m'),
                'post_flight_vertical_cov_reopen_hold_sec': LaunchConfiguration('post_flight_vertical_cov_reopen_hold_sec'),
                'post_flight_vertical_cov_reopen_grace_sec': LaunchConfiguration('post_flight_vertical_cov_reopen_grace_sec'),
                'post_flight_vertical_cov_reopen_pos_std_m': LaunchConfiguration('post_flight_vertical_cov_reopen_pos_std_m'),
                'post_flight_vertical_cov_reopen_vel_std_mps': LaunchConfiguration('post_flight_vertical_cov_reopen_vel_std_mps'),
                'post_flight_vertical_cov_reopen_accbias_std_z_mps2': LaunchConfiguration('post_flight_vertical_cov_reopen_accbias_std_z_mps2'),
                'terminal_descent_observation_enable': ParameterValue(LaunchConfiguration('terminal_descent_observation_enable'), value_type=bool),
                'terminal_descent_require_rtl_mode': ParameterValue(LaunchConfiguration('terminal_descent_require_rtl_mode'), value_type=bool),
                'terminal_descent_max_horizontal_speed_mps': ParameterValue(LaunchConfiguration('terminal_descent_max_horizontal_speed_mps'), value_type=float),
                'terminal_descent_min_vertical_speed_mps': ParameterValue(LaunchConfiguration('terminal_descent_min_vertical_speed_mps'), value_type=float),
                'terminal_descent_max_gyro_deg_s': ParameterValue(LaunchConfiguration('terminal_descent_max_gyro_deg_s'), value_type=float),
                'terminal_descent_max_source_yaw_rate_deg_s': ParameterValue(LaunchConfiguration('terminal_descent_max_source_yaw_rate_deg_s'), value_type=float),
                'terminal_descent_min_armed_time_sec': ParameterValue(LaunchConfiguration('terminal_descent_min_armed_time_sec'), value_type=float),
                'terminal_descent_native_gnss_vel_override_enable': ParameterValue(LaunchConfiguration('terminal_descent_native_gnss_vel_override_enable'), value_type=bool),
                'terminal_descent_native_gnss_vel_std_h_mps': ParameterValue(LaunchConfiguration('terminal_descent_native_gnss_vel_std_h_mps'), value_type=float),
                'terminal_descent_native_gnss_vel_std_u_mps': ParameterValue(LaunchConfiguration('terminal_descent_native_gnss_vel_std_u_mps'), value_type=float),
                'terminal_descent_vertical_cov_reopen_enable': ParameterValue(LaunchConfiguration('terminal_descent_vertical_cov_reopen_enable'), value_type=bool),
                'terminal_descent_vertical_cov_reopen_pos_std_m': ParameterValue(LaunchConfiguration('terminal_descent_vertical_cov_reopen_pos_std_m'), value_type=float),
                'terminal_descent_vertical_cov_reopen_vel_std_mps': ParameterValue(LaunchConfiguration('terminal_descent_vertical_cov_reopen_vel_std_mps'), value_type=float),
                'terminal_descent_vertical_cov_reopen_accbias_std_z_mps2': ParameterValue(LaunchConfiguration('terminal_descent_vertical_cov_reopen_accbias_std_z_mps2'), value_type=float),
                'terminal_descent_horizontal_zero_vel_enable': ParameterValue(LaunchConfiguration('terminal_descent_horizontal_zero_vel_enable'), value_type=bool),
                'terminal_descent_horizontal_zero_vel_max_hspeed_mps': ParameterValue(LaunchConfiguration('terminal_descent_horizontal_zero_vel_max_hspeed_mps'), value_type=float),
                'terminal_descent_horizontal_zero_vel_std_h_mps': ParameterValue(LaunchConfiguration('terminal_descent_horizontal_zero_vel_std_h_mps'), value_type=float),
                'terminal_descent_horizontal_zero_vel_std_u_mps': ParameterValue(LaunchConfiguration('terminal_descent_horizontal_zero_vel_std_u_mps'), value_type=float),
                'tilt_force_relock_min_residual_deg': LaunchConfiguration('tilt_force_relock_min_residual_deg'),
                'tilt_force_relock_roll_pitch_std_deg': LaunchConfiguration('tilt_force_relock_roll_pitch_std_deg'),
                'tilt_force_relock_once_per_motion_context': LaunchConfiguration('tilt_force_relock_once_per_motion_context'),
                'tilt_force_relock_max_rate_hz': LaunchConfiguration('tilt_force_relock_max_rate_hz'),
                # RViz Path gating: disarmed 时不画 Path，避免起飞前"蜘蛛网"
                'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
                'mavros_local_velocity_topic': LaunchConfiguration('mavros_local_velocity_topic'),
                'speed_source': LaunchConfiguration('speed_source'),
                'px4_vehicle_local_position_topic': LaunchConfiguration('px4_vehicle_local_position_topic'),
                'px4_vehicle_odometry_topic': LaunchConfiguration('px4_vehicle_odometry_topic'),
                'heading_source': LaunchConfiguration('heading_source'),
                'mavros_heading_topic': LaunchConfiguration('mavros_heading_topic'),
                'px4_vehicle_attitude_topic': LaunchConfiguration('px4_vehicle_attitude_topic'),
                'path_require_armed': True,
                'clear_path_on_arm_transition': False,
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_kf_gins')),
    )
    kf_gins_delayed = TimerAction(
        period=LaunchConfiguration('kf_gins_start_delay_sec'),
        actions=[kf_gins],
    )

    # ============ 实时对比分析 ============
    real_time_comparison = Node(
        package='kf_gins_ros2_native',
        executable='real_time_comparison.py',  # ⭐ 新脚本
        name='real_time_comparison',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ekf2_pose_topic': '/ekf2/pose',
            'ekf2_odom_topic': '/ekf2/pose_odom',
            'subscribe_ekf2_pose': LaunchConfiguration('comparison_subscribe_ekf2_pose'),
            'iekf_topic': '/kf_gins/odom',
            'iekf_raw_topic': LaunchConfiguration('iekf_raw_topic'),
            'compute_raw_metrics': LaunchConfiguration('comparison_compute_raw_metrics'),
            'raw_callback_mode': LaunchConfiguration('comparison_raw_callback_mode'),
            'iekf_fallback_topic': LaunchConfiguration('iekf_fallback_topic'),
            'comparison_output': '/comparison/metrics',
            'metrics_csv_path': LaunchConfiguration('comparison_csv_path'),
            'metrics_publish_rate': LaunchConfiguration('comparison_metrics_publish_rate'),
            'metrics_log_period_sec': LaunchConfiguration('comparison_metrics_log_period_sec'),
            'sync_tolerance_ms': 50,
            'align_initial': True,
            'max_abs_position_m': 10000.0,
            'max_abs_velocity_mps': 100.0,
            'max_pose_jump_m': 50.0,
            'max_velocity_jump_mps': 100.0,
            'recompute_alignment_on_jump': True,
            'clear_error_history_on_realign': True,
            'publish_live_metrics': LaunchConfiguration('comparison_publish_live_metrics'),
            'publish_named_metrics': LaunchConfiguration('comparison_publish_named_metrics'),
            'aligned_fallback_raw': LaunchConfiguration('aligned_fallback_raw'),
            'publish_iekf_state': LaunchConfiguration('publish_iekf_state'),
            'publish_ekf2_state': LaunchConfiguration('publish_ekf2_state'),
            'publish_aligned_iekf_state': LaunchConfiguration('publish_aligned_iekf_state'),
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_real_time_comparison'),
            "' == 'true' and '",
            LaunchConfiguration('enable_kf_gins'),
            "' == 'true' and '",
            LaunchConfiguration('enable_ekf2_relay'),
            "' == 'true'",
        ]))
    )

    iekf_aligned_path = Node(
        package='kf_gins_ros2_native',
        executable='iekf_aligned_path_publisher.py',
        name='iekf_aligned_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/iekf/state_aligned/position',
            'output_topic': '/iekf/path_aligned',
            'frame_id': 'map',
            'max_path_length': 20000,
            'require_armed': LaunchConfiguration('aligned_path_require_armed'),
            'clear_on_arm_transition': False,
            'clear_on_reset_event': True,
            'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
            'mavros_state_stale_sec': 1.5,
            'armed_false_hold_sec': 6.0,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_real_time_comparison'),
            "' == 'true' and '",
            LaunchConfiguration('enable_iekf_aligned_path'),
            "' == 'true'",
        ]))
    )

    # ============ ROS Bag 记录器 ============
    # 使用命令行工具而非 ros2bag Node (更稳定)
    bag_dir = f"/tmp/kf_gins_comparison_{run_tag}"
    
    # rosbag 会自动创建输出目录
    record_bag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', bag_dir,
             '/clock',
             '/tf',
             '/tf_static',
             '/imu/data',
             '/gps/fix',
             '/gps_dropzones/markers',
             '/mavros/gps_input',
             '/mavros/hil/gps',
             '/ekf2/pose',
             '/ekf2/pose_odom',
             '/ekf2/path',
             '/ekf2/state/position',
             '/ekf2/state/velocity',
             '/ekf2/state/rpy',
             '/kf_gins/odom',
             '/kf_gins/odom_raw',
             '/kf_gins/fallback_active',
             '/kf_gins/path',
             '/comparison/metrics',
             '/comparison/pos_err',
             '/comparison/pos_rmse',
             '/comparison/pos_mae',
             '/comparison/vel_err',
             '/comparison/att_err',
             '/comparison/yaw_err',
             '/comparison/pos_err_xyz',
             '/comparison/vel_err_xyz',
             '/comparison/att_err_rpy',
             '/comparison/sync_dt',
             '/comparison/fallback_active',
             '/comparison/ekf2_initialized',
             '/comparison/iekf_initialized',
             '/iekf/state/position',
             '/iekf/state/velocity',
             '/iekf/state/rpy',
             '/iekf/state_aligned/position',
             '/iekf/state_aligned/velocity',
             '/iekf/state_aligned/rpy',
             '/iekf/path_aligned',
             '/mavros/battery'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    # ============ RViz 可视化 ============
    rviz_config = PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'),
        'rviz',
        'ekf2_vs_iekf.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_comparison',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    # ============ 日志指导 ============
    log_info = LogInfo(
        msg="=" * 80 +
        "\n🚀 EKF2 vs IEKF 对比测试已启动\n" +
        "=" * 80 +
        "\n📊 监控信息:\n" +
        "  - EKF2 输出: /ekf2/pose\n" +
        "  - IEKF 输出: /kf_gins/odom\n" +
        "  - IEKF raw 输出: /kf_gins/odom_raw\n" +
        "  - fallback 标志: /kf_gins/fallback_active\n" +
        "  - 对比指标(误差): /comparison/metrics (Float32MultiArray, legacy)\n" +
        "  - 对比指标(命名): /comparison/pos_err /comparison/yaw_err /comparison/fallback_active ...\n" +
        "  - 状态(PlotJuggler): /ekf2/state/*  /iekf/state/*  /iekf/state_aligned/*\n" +
        "  - GNSS 遮挡: enable_gps_dropzones:=true (见 /gps_dropzones/markers)\n" +
        "  - GNSS 注入: px4_gps_injection_mode:=hil_gps|gps_input\n" +
        "  - PX4 参数: MAV_USEHILGPS=1 (px4_set_params:=true)\n" +
        "\n🔍 查看实时数据:\n" +
        "  rqt_plot /comparison/metrics/data[0]  # position_error_norm\n" +
        "  rqt_plot /comparison/metrics/data[1]  # attitude_error_norm\n" +
        "  rqt_plot /comparison/metrics/data[2]  # velocity_error_norm\n" +
        "  rqt_plot /comparison/pos_err  /comparison/yaw_err  /comparison/vel_err\n" +
        "\n📈 离线分析 (测试后):\n" +
        "  python3 ~/kf_gins_ws/src/kf_gins_ros2_native/scripts/offline_analysis.py\n" +
        f"\n📁 rosbag 输出目录:\n  {bag_dir}\n" +
        f"\n🧾 对比 CSV 输出:\n  {os.path.join('/tmp', f'kf_gins_comparison_{run_tag}', 'comparison_metrics.csv')}\n" +
        "\n" + "=" * 80
    )

    # ============ RQT 配置 (可选) ============
    # 自动启动数据绘图工具（可选，可通过手动启动）
    # 使用方式: rqt_plot /comparison/metrics/position_error

    # ============ 配置环境变量 (避免 RViz 显示问题) ============
    set_env = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        # Qt/rviz 在虚拟机或 XWayland 下可能出现“闪烁/黑条”：
        # 禁用 MIT-SHM 共享内存通常能显著改善（尤其是 VirtualBox/VMware/远程桌面环境）。
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        # 注意：强制软件渲染会让 Gazebo/RViz 极度卡顿（尤其是复杂 world）。
        # 如果你的环境必须用软件渲染再手动 export：
        #   export LIBGL_ALWAYS_SOFTWARE=1 QT_OPENGL=software
        SetEnvironmentVariable('LIBGL_DRI3_DISABLE', '1'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [EnvironmentVariable('LD_LIBRARY_PATH'),
             TextSubstitution(text=':/opt/ros/humble/opt/rviz_ogre_vendor/lib')]
        ),
    ]

    # ============ 组合所有节点 ============
    return LaunchDescription([
        # 参数声明
        use_sim_time,
        kf_gins_param_file,
        kf_gins_core_config_file,
        force_zero_antlever,
        use_gnss_llh_for_pose,
        scenario,
        record_bag,
        start_rviz,
        enable_real_time_comparison,
        publish_ekf2_state,
        publish_iekf_state,
        publish_aligned_iekf_state,
        comparison_publish_named_metrics,
        comparison_publish_live_metrics,
        comparison_metrics_publish_rate,
        comparison_metrics_log_period_sec,
        iekf_raw_topic,
        comparison_compute_raw_metrics,
        comparison_raw_callback_mode,
        iekf_fallback_topic,
        comparison_csv_path,
        gnss_update_debug_csv_path,
        gnss_nis_debug_csv_path,
        gnss_nis_debug_max_rate_hz,
        state_update_debug_csv_path,
        state_update_debug_max_rate_hz,
        dtrq_runtime_feature_debug_csv_path,
        dtrq_runtime_feature_debug_max_rate_hz,
        early_recovery_bias_feedback_debug_enable,
        early_recovery_bias_feedback_apply_enable,
        early_recovery_bias_feedback_history_sec,
        early_recovery_bias_feedback_min_armed_time_sec,
        early_recovery_bias_feedback_max_armed_time_sec,
        early_recovery_bias_feedback_ba_z_mean_max_mps2,
        early_recovery_bias_feedback_residual_u_mean_max_m,
        early_recovery_bias_feedback_core_gnss_u_mean_min_m,
        early_recovery_bias_feedback_dx_ba_z_sum_max_mps2,
        early_recovery_bias_feedback_min_history_rows,
        early_recovery_bias_feedback_negative_dx_scale,
        horizontal_consistency_debug_csv_path,
        heading_update_debug_csv_path,
        state_publish_debug_csv_path,
        shadow_restore_publish_enable,
        shadow_restore_subscribe_enable,
        shadow_restore_topic,
        shadow_restore_publish_after_core_sec,
        shadow_restore_publish_once,
        shadow_restore_publish_period_sec,
        shadow_restore_max_age_sec,
        shadow_restore_covariance_inflation_factor,
        shadow_restore_require_core_initialized,
        shadow_restore_clear_path_on_apply,
        shadow_restore_event_csv_path,
        shadow_supervisor_fsm_debug_enable,
        shadow_supervisor_fsm_reference_odom_topic,
        shadow_supervisor_fsm_debug_csv_path,
        shadow_supervisor_fsm_events_csv_path,
        shadow_supervisor_fsm_debug_max_rate_hz,
        shadow_supervisor_fsm_reference_odom_max_age_sec,
        shadow_supervisor_fsm_gamma_guard_enter,
        shadow_supervisor_fsm_gamma_shadow_max,
        shadow_supervisor_fsm_process_lambda_enter,
        shadow_supervisor_fsm_process_score_min,
        shadow_supervisor_fsm_warmup_sec,
        shadow_supervisor_fsm_ready_confirm_sec,
        shadow_supervisor_fsm_xy_delta_ready_max_m,
        shadow_supervisor_fsm_z_delta_ready_max_m,
        shadow_supervisor_fsm_vel_delta_ready_max_mps,
        shadow_supervisor_fsm_yaw_delta_ready_max_deg,
        shadow_supervisor_fsm_allow_mixed_trigger,
        shadow_supervisor_fsm_observation_score_guard_enable,
        shadow_supervisor_fsm_observation_score_guard_enter,
        shadow_supervisor_perf_proxy_publish_enable,
        shadow_supervisor_perf_proxy_subscribe_enable,
        shadow_supervisor_perf_proxy_topic,
        shadow_supervisor_perf_proxy_max_age_sec,
        shadow_supervisor_perf_proxy_short_window_sec,
        shadow_supervisor_perf_proxy_long_window_sec,
        shadow_supervisor_perf_proxy_residual_gap_soft_max_m,
        shadow_supervisor_perf_proxy_nis_gap_permission_max,
        shadow_supervisor_predictive_score_publish_enable,
        shadow_supervisor_predictive_score_subscribe_enable,
        shadow_supervisor_predictive_score_debug_enable,
        shadow_supervisor_predictive_score_topic,
        shadow_supervisor_predictive_score_debug_csv_path,
        shadow_supervisor_predictive_score_source_id,
        shadow_supervisor_predictive_score_max_age_sec,
        shadow_supervisor_predictive_score_stamp_tolerance_sec,
        shadow_supervisor_predictive_score_short_window_sec,
        shadow_supervisor_predictive_score_long_window_sec,
        shadow_supervisor_predictive_score_eval_std_h_m,
        shadow_supervisor_predictive_score_eval_std_u_m,
        shadow_supervisor_velocity_predictive_score_publish_enable,
        shadow_supervisor_velocity_predictive_score_subscribe_enable,
        shadow_supervisor_velocity_predictive_score_debug_enable,
        shadow_supervisor_velocity_predictive_score_topic,
        shadow_supervisor_velocity_predictive_score_debug_csv_path,
        shadow_supervisor_velocity_predictive_score_source_id,
        shadow_supervisor_velocity_predictive_score_max_age_sec,
        shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec,
        shadow_supervisor_velocity_predictive_score_short_window_sec,
        shadow_supervisor_velocity_predictive_score_long_window_sec,
        shadow_supervisor_velocity_predictive_score_eval_std_h_mps,
        shadow_supervisor_velocity_predictive_score_eval_std_u_mps,
        shadow_supervisor_kinematic_predictive_score_publish_enable,
        shadow_supervisor_kinematic_predictive_score_subscribe_enable,
        shadow_supervisor_kinematic_predictive_score_debug_enable,
        shadow_supervisor_kinematic_predictive_score_topic,
        shadow_supervisor_kinematic_predictive_score_debug_csv_path,
        shadow_supervisor_kinematic_predictive_score_source_id,
        shadow_supervisor_kinematic_predictive_score_max_age_sec,
        shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec,
        shadow_supervisor_kinematic_predictive_score_short_window_sec,
        shadow_supervisor_kinematic_predictive_score_long_window_sec,
        shadow_supervisor_kinematic_predictive_score_eval_std_h_m,
        shadow_supervisor_kinematic_predictive_score_eval_std_u_m,
        shadow_supervisor_kinematic_predictive_score_eval_std_h_mps,
        shadow_supervisor_kinematic_predictive_score_eval_std_u_mps,
        segment_timing_gate_debug_csv_path,
        segment_timing_gate_segment_sec,
        segment_timing_gate_lag_threshold_sec,
        segment_timing_gate_gnss_source_age_max_sec,
        segment_timing_gate_core_gnss_along_min_enable,
        segment_timing_gate_core_gnss_along_min_threshold_m,
        segment_timing_gate_projection_alpha,
        segment_timing_gate_projection_enable,
        segment_timing_gate_projection_apply_mission,
        segment_timing_gate_projection_apply_rtl,
        segment_timing_gate_projection_apply_other,
        publish_state_after_gnss_update,
        publish_px4_sphere_projection,
        accbias_z_history_projection_enable,
        accbias_z_history_projection_apply_mission,
        accbias_z_history_projection_apply_rtl,
        accbias_z_history_projection_apply_other,
        accbias_z_history_projection_alpha,
        accbias_z_history_projection_deep_threshold_mps2,
        accbias_z_history_projection_frac_threshold,
        accbias_z_history_projection_history_start_sec,
        publish_stamp_mode,
        publish_core_stamp_max_future_sec,
        publish_core_stamp_max_past_sec,
        publish_core_stamp_offset_bias_sec,
        raw_odom_decimation,
        path_publish_rate_hz,
        pose_decimation,
        max_path_points,
        core_processing_enable,
        core_imu_decimation,
        core_max_imu_rate_hz,
        armed_cruise_native_gnss_vel_override_enable,
        armed_cruise_gnss_pos_override_enable,
        armed_cruise_gnss_pos_override_apply_mission,
        armed_cruise_gnss_pos_override_apply_rtl,
        armed_cruise_gnss_pos_override_apply_other,
        armed_cruise_gnss_pos_std_h_m,
        armed_cruise_gnss_pos_std_u_m,
        armed_cruise_gnss_pos_residual_boost_enable,
        armed_cruise_gnss_pos_residual_boost_threshold_m,
        armed_cruise_gnss_pos_residual_boost_hold_sec,
        gnss_position_response_boost_enable,
        gnss_position_response_boost_apply_mission,
        gnss_position_response_boost_apply_rtl,
        gnss_position_response_boost_residual_start_h_m,
        gnss_position_response_boost_residual_full_h_m,
        gnss_position_response_boost_min_horizontal_speed_mps,
        gnss_position_response_boost_max_abs_vertical_speed_mps,
        gnss_position_response_boost_persistence_updates,
        gnss_position_response_boost_std_min_h_m,
        gnss_position_response_boost_std_max_h_m,
        gnss_position_gain_response_enable,
        gnss_position_gain_response_apply_mission,
        gnss_position_gain_response_apply_rtl,
        gnss_position_gain_response_require_armed_cruise,
        gnss_position_gain_response_block_turning,
        gnss_position_gain_response_block_post_turn,
        gnss_position_gain_response_residual_start_h_m,
        gnss_position_gain_response_residual_full_h_m,
        gnss_position_gain_response_hnis_start,
        gnss_position_gain_response_hnis_full,
        gnss_position_gain_response_prev_gain_low,
        gnss_position_gain_response_prev_gain_high,
        gnss_position_gain_response_min_horizontal_speed_mps,
        gnss_position_gain_response_max_abs_vertical_speed_mps,
        gnss_position_gain_response_persistence_updates,
        gnss_position_gain_response_std_min_h_m,
        gnss_position_gain_response_std_max_h_m,
        gnss_velocity_outward_damping_enable,
        gnss_velocity_outward_damping_apply_mission,
        gnss_velocity_outward_damping_apply_rtl,
        gnss_velocity_outward_damping_min_core_gnss_diff_h_m,
        gnss_velocity_outward_damping_radial_start_mps,
        gnss_velocity_outward_damping_radial_full_mps,
        gnss_velocity_outward_damping_min_horizontal_speed_mps,
        gnss_velocity_outward_damping_max_abs_vertical_speed_mps,
        gnss_velocity_outward_damping_persistence_updates,
        gnss_velocity_outward_damping_hold_updates,
        gnss_velocity_outward_damping_std_min_h_mps,
        gnss_velocity_outward_damping_std_max_h_mps,
        turn_postturn_native_velocity_deweight_enable,
        turn_postturn_native_velocity_deweight_apply_mission,
        turn_postturn_native_velocity_deweight_apply_rtl,
        turn_postturn_native_velocity_deweight_apply_turning,
        turn_postturn_native_velocity_deweight_apply_post_turn,
        turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m,
        turn_postturn_native_velocity_deweight_radial_abs_start_mps,
        turn_postturn_native_velocity_deweight_radial_abs_full_mps,
        turn_postturn_native_velocity_deweight_core_residual_start_h_mps,
        turn_postturn_native_velocity_deweight_core_residual_full_h_mps,
        turn_postturn_native_velocity_deweight_min_horizontal_speed_mps,
        turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps,
        turn_postturn_native_velocity_deweight_persistence_updates,
        turn_postturn_native_velocity_deweight_std_min_h_mps,
        turn_postturn_native_velocity_deweight_std_max_h_mps,
        phase_error_memory_debug_enable,
        phase_error_memory_debug_residual_threshold_h_m,
        phase_error_memory_debug_dx_over_residual_threshold,
        phase_error_memory_debug_recent_turnpost_hold_sec,
        adaptive_gnss_pos_weight_enable,
        adaptive_gnss_pos_weight_apply_mission,
        adaptive_gnss_pos_weight_apply_rtl,
        adaptive_gnss_pos_weight_trigger_source,
        adaptive_gnss_pos_weight_floor_min_h_m,
        adaptive_gnss_pos_weight_floor_nominal_h_m,
        adaptive_gnss_pos_weight_floor_max_h_m,
        adaptive_gnss_pos_weight_residual_start_h_m,
        adaptive_gnss_pos_weight_residual_full_h_m,
        adaptive_gnss_pos_weight_nis_start_h_2d,
        adaptive_gnss_pos_weight_nis_full_h_2d,
        adaptive_gnss_pos_weight_nis_max_age_sec,
        adaptive_gnss_pos_weight_persistence_updates,
        adaptive_gnss_pos_weight_attack_sec,
        adaptive_gnss_pos_weight_decay_sec,
        adaptive_gnss_pos_weight_armed_cruise_gain,
        adaptive_gnss_pos_weight_turn_gain,
        adaptive_gnss_pos_weight_post_turn_gain,
        horizontal_consistency_supervisor_enable,
        horizontal_consistency_apply_mission,
        horizontal_consistency_apply_rtl,
        horizontal_consistency_nis_start_h_2d,
        horizontal_consistency_nis_full_h_2d,
        horizontal_consistency_residual_start_h_m,
        horizontal_consistency_residual_full_h_m,
        horizontal_consistency_core_gnss_start_h_m,
        horizontal_consistency_core_gnss_full_h_m,
        horizontal_consistency_heading_residual_start_deg,
        horizontal_consistency_heading_residual_full_deg,
        horizontal_consistency_score_trigger,
        horizontal_consistency_persistence_updates,
        horizontal_consistency_min_horizontal_speed_mps,
        horizontal_consistency_max_vertical_speed_mps,
        mission_cov_hygiene_enable,
        mission_cov_hygiene_apply_mission,
        mission_cov_hygiene_apply_rtl,
        mission_cov_hygiene_hnis_start,
        mission_cov_hygiene_hnis_full,
        mission_cov_hygiene_resid_start_h_m,
        mission_cov_hygiene_resid_full_h_m,
        mission_cov_hygiene_pos_std_tight_lo_h_m,
        mission_cov_hygiene_pos_std_tight_hi_h_m,
        mission_cov_hygiene_dx_ratio_low,
        mission_cov_hygiene_dx_ratio_high,
        mission_cov_hygiene_persistence_updates,
        mission_cov_hygiene_attack_sec,
        mission_cov_hygiene_decay_sec,
        mission_cov_hygiene_floor_min_h_m,
        mission_cov_hygiene_floor_nominal_h_m,
        mission_cov_hygiene_floor_max_h_m,
        mission_cov_hygiene_offdiag_corr_limit,
        turn_rate_propagation_noise_probe_enable,
        turn_rate_propagation_noise_probe_apply_mission,
        turn_rate_propagation_noise_probe_apply_rtl,
        turn_rate_propagation_noise_probe_gyro_start_deg_s,
        turn_rate_propagation_noise_probe_gyro_full_deg_s,
        turn_rate_propagation_noise_probe_min_horizontal_speed_mps,
        turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps,
        turn_rate_propagation_noise_probe_arw_q_scale_max,
        turn_rate_propagation_noise_probe_vrw_q_scale_max,
        turn_rate_propagation_noise_probe_gyrbias_q_scale_max,
        turn_rate_propagation_noise_probe_accbias_q_scale_max,
        accbias_z_propagation_probe_enable,
        accbias_z_propagation_probe_apply_noise_scale,
        accbias_z_propagation_probe_apply_mission,
        accbias_z_propagation_probe_apply_rtl,
        accbias_z_propagation_probe_trigger_mps2,
        accbias_z_propagation_probe_full_mps2,
        accbias_z_propagation_probe_min_horizontal_speed_mps,
        accbias_z_propagation_probe_max_abs_vertical_speed_mps,
        accbias_z_propagation_probe_arw_q_scale_max,
        accbias_z_propagation_probe_vrw_q_scale_max,
        accbias_z_propagation_probe_gyrbias_q_scale_max,
        accbias_z_propagation_probe_accbias_q_scale_max,
        motion_gnss_pos_weight_enable,
        motion_gnss_pos_weight_apply_mission,
        motion_gnss_pos_weight_apply_rtl,
        motion_gnss_pos_weight_hspeed_start_mps,
        motion_gnss_pos_weight_hspeed_full_mps,
        motion_gnss_pos_weight_max_abs_vspeed_mps,
        motion_gnss_pos_weight_std_min_h_m,
        motion_gnss_pos_weight_std_max_h_m,
        gnss_pos_recovery_weight_enable,
        gnss_pos_recovery_weight_apply_mission,
        gnss_pos_recovery_weight_apply_rtl,
        gnss_pos_recovery_weight_residual_start_h_m,
        gnss_pos_recovery_weight_residual_full_h_m,
        gnss_pos_recovery_weight_core_start_h_m,
        gnss_pos_recovery_weight_core_full_h_m,
        gnss_pos_recovery_weight_core_score_gain,
        gnss_pos_recovery_weight_min_horizontal_speed_mps,
        gnss_pos_recovery_weight_max_abs_vertical_speed_mps,
        gnss_pos_recovery_weight_persistence_updates,
        gnss_pos_recovery_weight_hold_sec,
        gnss_pos_recovery_weight_attack_sec,
        gnss_pos_recovery_weight_decay_sec,
        gnss_pos_recovery_weight_std_min_h_m,
        gnss_pos_recovery_weight_std_max_h_m,
        context_gnss_pos_floor_enable,
        context_gnss_pos_floor_apply_mission,
        context_gnss_pos_floor_apply_rtl,
        context_gnss_pos_floor_mission_base_h_m,
        context_gnss_pos_floor_turn_post_h_m,
        context_gnss_pos_floor_armed_cruise_h_m,
        context_gnss_pos_floor_rtl_h_m,
        context_gnss_pos_floor_attack_sec,
        context_gnss_pos_floor_decay_sec,
        heading_cruise_micro_track_enable,
        heading_cruise_micro_track_std_deg,
        heading_cruise_micro_track_min_residual_deg,
        heading_cruise_micro_track_max_residual_deg,
        heading_cruise_micro_track_min_horizontal_speed_mps,
        heading_cruise_micro_track_max_vertical_speed_mps,
        heading_cruise_micro_track_max_gyro_deg_s,
        heading_cruise_micro_track_max_source_yaw_rate_deg_s,
        heading_yaw_gain_hygiene_enable,
        heading_yaw_gain_hygiene_apply_mission,
        heading_yaw_gain_hygiene_apply_rtl,
        heading_yaw_gain_hygiene_apply_armed_cruise,
        heading_yaw_gain_hygiene_apply_post_turn,
        heading_yaw_gain_hygiene_apply_turn,
        heading_yaw_gain_hygiene_apply_update,
        heading_yaw_gain_hygiene_yaw_std_floor_deg,
        heading_yaw_gain_hygiene_min_residual_deg,
        heading_yaw_gain_hygiene_max_residual_deg,
        heading_yaw_gain_hygiene_min_horizontal_speed_mps,
        heading_yaw_gain_hygiene_max_abs_vertical_speed_mps,
        heading_yaw_gain_hygiene_max_gyro_deg_s,
        heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s,
        heading_yaw_gain_hygiene_max_rate_hz,
        armed_cruise_native_gnss_vel_min_horizontal_speed_mps,
        armed_cruise_native_gnss_vel_std_h_mps,
        armed_cruise_native_gnss_vel_std_u_mps,
        armed_cruise_native_gnss_vel_residual_boost_enable,
        armed_cruise_native_gnss_vel_residual_boost_threshold_mps,
        armed_cruise_native_gnss_vel_residual_boost_hold_sec,
        armed_cruise_native_gnss_vel_residual_boost_std_h_mps,
        armed_cruise_native_gnss_vel_residual_boost_std_u_mps,
        native_gnss_velocity_outlier_guard_enable,
        native_gnss_velocity_outlier_guard_apply_mission,
        native_gnss_velocity_outlier_guard_apply_rtl,
        native_gnss_velocity_outlier_guard_speed_mismatch_mps,
        native_gnss_velocity_outlier_guard_core_residual_h_mps,
        native_gnss_velocity_outlier_guard_min_horizontal_speed_mps,
        native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps,
        native_gnss_velocity_outlier_guard_apply_turning_context,
        native_gnss_velocity_outlier_guard_action,
        native_gnss_velocity_outlier_guard_reweight_std_h_mps,
        native_gnss_velocity_outlier_guard_reweight_std_u_mps,
        native_gnss_velocity_low_speed_turn_source_guard_enable,
        native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps,
        native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s,
        native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps,
        armed_cruise_vertical_cov_reopen_enable,
        armed_cruise_vertical_cov_reopen_threshold_m,
        armed_cruise_vertical_cov_reopen_hold_sec,
        armed_cruise_vertical_cov_reopen_pos_std_m,
        armed_cruise_vertical_cov_reopen_vel_std_mps,
        armed_cruise_vertical_cov_reopen_accbias_std_z_mps2,
        post_flight_vertical_cov_reopen_enable,
        post_flight_vertical_cov_reopen_threshold_m,
        post_flight_vertical_cov_reopen_hold_sec,
        post_flight_vertical_cov_reopen_grace_sec,
        post_flight_vertical_cov_reopen_pos_std_m,
        post_flight_vertical_cov_reopen_vel_std_mps,
        post_flight_vertical_cov_reopen_accbias_std_z_mps2,
        tilt_force_relock_min_residual_deg,
        tilt_force_relock_roll_pitch_std_deg,
        tilt_force_relock_once_per_motion_context,
        tilt_force_relock_max_rate_hz,
        aligned_fallback_raw,
        ekf2_use_input_stamp,
        ekf2_relay_publish_pose,
        imu_use_node_stamp,
        enable_imu_converter,
        mavros_imu_topic,
        imu_source,
        px4_sensor_combined_topic,
        px4_vehicle_imu_topic,
        px4_imu_qos_depth,
        comparison_subscribe_ekf2_pose,
        mavros_gps_topic,
        gnss_relay_mode,
        enable_gnss_relay,
        gnss_source,
        px4_sensor_gps_topic,
        px4_vehicle_global_position_topic,
        mavros_local_pose_topic,
        ekf2_input_mode,
        px4_vehicle_odometry_topic,
        enable_px4_aux_state_relay,
        px4_aux_imu_topic,
        px4_aux_local_velocity_topic,
        mavros_local_velocity_topic,
        speed_source,
        px4_vehicle_local_position_topic,
        mavros_state_topic,
        heading_source,
        mavros_heading_topic,
        px4_vehicle_attitude_topic,
        mavros_hil_gps_topic,
        gnss_relay_start_delay_sec,
        gnss_relay_publish_enable,
        gnss_relay_subscribe_enable,
        gnss_relay_publish_min_interval_sec,
        gnss_disturbance_enable,
        gnss_disturbance_trace_csv,
        gnss_disturbance_log_csv,
        gnss_disturbance_time_offset_sec,
        enable_gps_dropzones,
        inject_dropzone_gps_to_px4,
        px4_gps_injection_mode,
        px4_set_params,
        px4_param_service_v2,
        px4_param_service,
        use_sim_gnss_std,
        sim_gnss_std_h_m,
        sim_gnss_std_u_m,
        gnss_position_lag_compensation_enable,
        gnss_position_lag_compensation_sec,
        gnss_position_lag_compensation_max_sec,
        gnss_position_lag_compensation_min_speed_mps,
        terminal_descent_observation_enable,
        terminal_descent_require_rtl_mode,
        terminal_descent_max_horizontal_speed_mps,
        terminal_descent_min_vertical_speed_mps,
        terminal_descent_max_gyro_deg_s,
        terminal_descent_max_source_yaw_rate_deg_s,
        terminal_descent_min_armed_time_sec,
        terminal_descent_native_gnss_vel_override_enable,
        terminal_descent_native_gnss_vel_std_h_mps,
        terminal_descent_native_gnss_vel_std_u_mps,
        terminal_descent_vertical_cov_reopen_enable,
        terminal_descent_vertical_cov_reopen_pos_std_m,
        terminal_descent_vertical_cov_reopen_vel_std_mps,
        terminal_descent_vertical_cov_reopen_accbias_std_z_mps2,
        terminal_descent_horizontal_zero_vel_enable,
        terminal_descent_horizontal_zero_vel_max_hspeed_mps,
        terminal_descent_horizontal_zero_vel_std_h_mps,
        terminal_descent_horizontal_zero_vel_std_u_mps,
        aligned_path_require_armed,
        enable_ekf2_path,
        enable_iekf_path,
        enable_gt_path,
        enable_ekf2_relay,
        enable_kf_gins,
        kf_gins_start_delay_sec,
        enable_iekf_aligned_path,
        enable_static_tf,
        
        # 日志输出
        log_info,
        
        # 基础设施
        static_tf,
        imu_convert,
        px4_aux_state_relay,
        gnss_relay_delayed,
        gps_dropzones,
        px4_param_setter,
        hil_gps_relay,
        gps_input_relay,
        
        # 轨迹转换
        ekf2_path,
        iekf_path,
        gt_path,
        ekf2_relay,
        
        # IEKF
        kf_gins_delayed,
        
        # 对比
        real_time_comparison,
        iekf_aligned_path,
        
        # 可视化和记录
        *set_env,
        record_bag_node,
        rviz,
        # rqt_launcher_script,  # 可选，可能需要单独处理
        
        # 输出提示
        LogInfo(msg="\n✅ 所有节点已启动，请检查 RViz 和终端输出\n")
    ])
