using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

// PID制御器クラス
[System.Serializable]
public class PIDController
{
    public float Kp = 1.0f; // 比例ゲイン
    public float Ki = 0.0f; // 積分ゲイン
    public float Kd = 0.1f; // 微分ゲイン
    
    private float integral = 0f; // 積分項
    private float lastError = 0f; // 前回の誤差
    private float maxIntegral = 100f; // 積分項の最大値（ウィンドウスティング防止）
    private float minIntegral = -100f; // 積分項の最小値
    
    public PIDController(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    
    // PID制御の計算
    public float Calculate(float target, float current, float deltaTime)
    {
        if (deltaTime <= 0f) return 0f;
        
        float error = target - current;
        
        // 比例項
        float proportional = Kp * error;
        
        // 積分項（ウィンドウスティング防止）
        integral += error * deltaTime;
        integral = Mathf.Clamp(integral, minIntegral, maxIntegral);
        float integralTerm = Ki * integral;
        
        // 微分項
        float derivative = (error - lastError) / deltaTime;
        float derivativeTerm = Kd * derivative;
        
        lastError = error;
        
        return proportional + integralTerm + derivativeTerm;
    }
    
    // PID制御器をリセット
    public void Reset()
    {
        integral = 0f;
        lastError = 0f;
    }
}

public class Car : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float maxRPM = 50f; // 最大回転数（RPM）
    public float rpmLimitStart = 50f; // 回転数制限開始点（RPM）
    public float maxBrakeTorque = 200f; // 最大ブレーキトルク
    public float reverseRPMMultiplier = 1.5f; // 後退時の回転数制限倍率
    public float autoBrakeThreshold = 2.0f; // 自動ブレーキ開始閾値（入力の80%以上）
    public float highSpeedSteeringMultiplier = 1.0f; // 高速時のステアリング倍率
    public bool allowReverse = false; // 後退入力・後退トルクを許可するか
    public bool sanitizeWheelVisualPhysics = true; // 見た目用タイヤの物理コンポーネントを無効化する
    public bool keepWheelModelsOnSteeringAxis = true; // タイヤモデルをステアリング軸のローカル空間に固定する
    public bool parentWheelModelsToSteeringAxis = true; // 実行時にタイヤモデルをステアリング軸の子にする
    public bool parentSteeringModelsToRocker = true; // ステアリング軸をロッカーピボットの子にする
    public bool addForwardForceWhenTurningOnly = false; // 旋回入力だけのときに微小な前進力を足すか
    public float turnOnlyForwardInput = 0.2f; // addForwardForceWhenTurningOnly が有効な場合の前進入力
    public float linearInputDeadzone = 0.05f; // この値未満の前後入力は停止指令として扱う
    public bool clampForwardVelocityWhenStopped = true; // 停止指令中に車体前後方向の微小速度を消す
    public float stoppedVelocityClampThreshold = 0.5f; // 停止指令中に速度を直接0へ寄せる上限（m/s）
    
    // ROS2制御の設定
    public bool useROS2Control = true; // ROS2制御を使用するかどうか
    public bool fallbackToKeyboard = false; // ROS2が利用できない場合にキーボードにフォールバックするか
    public float ros2Timeout = 0.1f; // ROS2データのタイムアウト時間（秒）- 100ms

    // ROS2関連の変数（rover_control.csから移植）
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> cmd_vel_sub;
    
    // 受信した制御データ（スレッドセーフなアクセス用）
    [HideInInspector]
    private float linearVelocity = 0f;
    [HideInInspector]
    private float angularVelocity = 0f;
    private readonly object velocityLock = new object(); // スレッドセーフティ用のロック
    
    // 制御データのスケーリング係数
    public float linearScale = 1.0f;
    public float angularScale = 1.0f;
    
    private Rigidbody carRigidbody;
    private float lastROS2DataTime = 0f;
    private bool ros2DataReceived = false; // ROS2データ受信フラグ
    private float lastSteering = 0f; // 前回のsteering値を保持
    private float lastMotor = 0f; // 前回のmotor値を保持
    private bool wasBraking = false; // 前回フレームでブレーキが適用されていたか
    private float brakeReleaseTime = -1f; // ブレーキが解除された時刻（-1は未初期化を意味する）
    public float motorRampUpTime = 0.3f; // ブレーキ解除後のモータートルク増加時間（秒）
    
    // PID制御の設定
    [Header("PID Control Settings")]
    public bool usePIDControl = true; // PID制御を使用するかどうか
    public float pidKp = 2.0f; // 比例ゲイン
    public float pidKi = 0.5f; // 積分ゲイン
    public float pidKd = 0.1f; // 微分ゲイン
    public float wheelRadius = 0.15f; // ホイールの半径（メートル）- RPMから速度への変換用
    public float speedHoldBrakeMultiplier = 1.0f; // 目標速度を超えたときのPIDブレーキ倍率
    public float vehicleSpeedBrakeGain = 250f; // 車体速度が目標を超えたときの追加ブレーキ倍率
    public float speedHoldTolerance = 0.05f; // 目標速度からこの値までは許容する（m/s）
    
    // Rocker機構の設定
    [Header("Rocker Suspension Settings")]
    public bool useRockerMechanism = true; // Rocker機構を使用するかどうか
    public float maxRockerAngle = 25f; // ロッカーアームの最大回転角度（度）
    public float rockerSmoothSpeed = 10f; // ロッカーアームの追従速度（大きいほど速く追従）
    public float rockerGain = 1.0f; // ロッカー角度のゲイン（回転方向が逆の場合は-1に）
    public bool useDifferentialBar = true; // 差動バー（左右のロッカー角度を車体で平均化）を使用するか
    
    // Rocker機構の内部状態
    private Dictionary<Transform, Quaternion> rockerInitialRotations = new Dictionary<Transform, Quaternion>(); // ピボットの初期回転
    private Dictionary<Transform, float> rockerCurrentAngles = new Dictionary<Transform, float>(); // 現在のロッカー角度
    private Dictionary<Transform, Vector3> visualInitialLocalPositions = new Dictionary<Transform, Vector3>();
    private Dictionary<Transform, Quaternion> visualInitialLocalRotations = new Dictionary<Transform, Quaternion>();
    
    // 各ホイール用のPID制御器
    private Dictionary<WheelCollider, PIDController> wheelPIDControllers = new Dictionary<WheelCollider, PIDController>();
    private float lastFixedUpdateTime = 0f; // 前回のFixedUpdateの時刻
    
    private float lastTimeoutLogTime = 0f; // 最後にタイムアウトログを出力した時刻
    private float timeoutLogInterval = 2.0f; // タイムアウトログの出力間隔（秒）
    private bool ros2NodeInitialized = false; // ROS2ノードが初期化されたかどうか
    private float lastBrakeLogTime = 0f; // 最後にブレーキログを出力した時刻
    private float brakeLogInterval = 1.0f; // ブレーキログの出力間隔（秒）
    
    void Start() {
        Application.targetFrameRate = 240;
        
        carRigidbody = GetComponent<Rigidbody>();
        lastFixedUpdateTime = Time.fixedTime;
        
        // PID制御器を初期化
        if (usePIDControl)
        {
            InitializePIDControllers();
        }
        
        // Rocker機構を初期化
        if (useRockerMechanism)
        {
            InitializeRockers();
        }

        if (sanitizeWheelVisualPhysics)
        {
            SanitizeWheelVisualPhysics();
        }

        InitializeWheelVisuals();
        
        // ROS2コンポーネントを取得
        if (useROS2Control)
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogWarning("ROS2UnityComponent not found. Falling back to keyboard control.");
                useROS2Control = false;
            }
        }
    }
    
    // PID制御器を初期化
    private void InitializePIDControllers()
    {
        wheelPIDControllers.Clear();
        
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.motor)
            {
                // 各ホイールにPID制御器を割り当て
                if (axleInfo.leftFrontWheel != null)
                    wheelPIDControllers[axleInfo.leftFrontWheel] = new PIDController(pidKp, pidKi, pidKd);
                if (axleInfo.rightFrontWheel != null)
                    wheelPIDControllers[axleInfo.rightFrontWheel] = new PIDController(pidKp, pidKi, pidKd);
                if (axleInfo.leftBackWheel != null)
                    wheelPIDControllers[axleInfo.leftBackWheel] = new PIDController(pidKp, pidKi, pidKd);
                if (axleInfo.rightBackWheel != null)
                    wheelPIDControllers[axleInfo.rightBackWheel] = new PIDController(pidKp, pidKi, pidKd);
            }
        }
        
        Debug.Log($"PID Controllers initialized for {wheelPIDControllers.Count} wheels");
    }
    
    // Rocker機構を初期化（各ピボットの初期回転を記録）
    private void InitializeRockers()
    {
        rockerInitialRotations.Clear();
        rockerCurrentAngles.Clear();
        
        int rockerCount = 0;
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.leftRockerPivot != null)
            {
                rockerInitialRotations[axleInfo.leftRockerPivot] = axleInfo.leftRockerPivot.localRotation;
                rockerCurrentAngles[axleInfo.leftRockerPivot] = 0f;
                rockerCount++;
            }
            if (axleInfo.rightRockerPivot != null)
            {
                rockerInitialRotations[axleInfo.rightRockerPivot] = axleInfo.rightRockerPivot.localRotation;
                rockerCurrentAngles[axleInfo.rightRockerPivot] = 0f;
                rockerCount++;
            }
        }
        
        if (rockerCount == 0)
        {
            Debug.LogWarning("Rocker mechanism enabled but no rocker pivots assigned. " +
                "Assign leftRockerPivot / rightRockerPivot in AxleInfo (parents of front/rear WheelColliders on each side).");
        }
        else
        {
            Debug.Log($"Rocker mechanism initialized with {rockerCount} pivots");
        }
    }
    
    void Update()
    {
        // ROS2ノードとサブスクリプションの初期化
        if (useROS2Control)
        {
            // ROS2が利用可能かチェック
            if (ros2Unity == null)
            {
                ros2Unity = GetComponent<ROS2UnityComponent>();
            }
            
            // ROS2が利用可能で、ノードが未初期化の場合
            if (ros2Unity != null && ros2Unity.Ok() && !ros2NodeInitialized)
            {
                try
                {
                    if (ros2Node == null)
                    {
                        ros2Node = ros2Unity.CreateNode("rover_control_sub");
                        cmd_vel_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                            "cmd_vel", OnCmdVelReceived);
                        ros2NodeInitialized = true;
                        Debug.Log("ROS2 Rover Control Subscriber initialized");
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"Failed to initialize ROS2 node: {e.Message}");
                    ros2NodeInitialized = false;
                }
            }
            // ROS2が利用不可能になった場合、ノードをリセット
            else if (ros2Unity != null && !ros2Unity.Ok() && ros2NodeInitialized)
            {
                Debug.LogWarning("ROS2 connection lost. Resetting node.");
                ros2Node = null;
                cmd_vel_sub = null;
                ros2NodeInitialized = false;
            }
        }
    }
    
    // cmd_velメッセージを受信した時のコールバック（rover_control.csから移植）
    private void OnCmdVelReceived(geometry_msgs.msg.Twist msg)
    {
        // スレッドセーフにデータを更新
        lock (velocityLock)
        {
            // 線形速度（前進・後退）
            linearVelocity = (float)(msg.Linear.X * linearScale);
            
            // 角速度（旋回）
            angularVelocity = (float)(-msg.Angular.Z * angularScale);
        }
        // ROS2データ受信フラグを設定
        ros2DataReceived = true;
    }
    
    public void FixedUpdate() {
        // デルタタイムを計算
        float deltaTime = Time.fixedTime - lastFixedUpdateTime;
        lastFixedUpdateTime = Time.fixedTime;
        
        // ROS2データ受信フラグをチェックして時間を更新（メインスレッドで実行）
        if (ros2DataReceived)
        {
            lastROS2DataTime = Time.time;
            ros2DataReceived = false;
        }
        
        float motor = 0f; // 初期値を0に設定
        float steering = lastSteering; // 前回の値を初期値として使用

        float brake = 0f;
        float targetForwardSpeed = 0f;
        bool holdStopped = false;
        
        // 入力ソースを決定（ROS2が利用可能で、データがタイムアウトしていない場合）
        bool ros2Available = useROS2Control && 
                            ros2Unity != null && 
                            ros2Unity.Ok() && 
                            ros2NodeInitialized;
        bool useROS2 = ros2Available && 
                      (Time.time - lastROS2DataTime) < ros2Timeout;
        
        if (useROS2)
        {
            // ROS2からの制御データを使用（スレッドセーフに取得）
            float linearInput, angularInput;
            lock (velocityLock)
            {
                linearInput = linearVelocity;
                angularInput = angularVelocity;
            }
            if (!allowReverse)
            {
                linearInput = Mathf.Max(0f, linearInput);
            }
            if (Mathf.Abs(linearInput) < linearInputDeadzone)
            {
                linearInput = 0f;
            }

            // デッドゾーンの処理を改善
            // 角速度が大きい場合でも、線形入力が小さい場合はそのまま使用
            // ただし、旋回のみの場合は最小限の前進力を追加
            if (addForwardForceWhenTurningOnly && Mathf.Abs(angularInput) > 0.5f && Mathf.Abs(linearInput) < 0.02f)
            {
                // 旋回のみの場合、最小限の前進力を追加（オプション）
                linearInput = turnOnlyForwardInput;
            }
            targetForwardSpeed = linearInput;

            // ブレーキが必要かどうかを先にチェック
            if (Mathf.Abs(linearInput) <= 0f)
            {
                // 停止する必要がある場合、モータートルクを0にしてブレーキを適用
                motor = 0f;
                brake = maxBrakeTorque; // ブレーキトルクを適切な値に修正
                holdStopped = true;
                // ブレーキが適用されたことを記録
                if (!wasBraking)
                {
                    wasBraking = true;
                    // ブレーキが初めて適用された時のみログを出力
                    Debug.Log($"Brake applied: {brake} (linearInput: {linearInput:F3})");
                }
                // ブレーキが継続している場合は、定期的にのみログを出力
                else
                {
                    float timeSinceLastBrakeLog = Time.time - lastBrakeLogTime;
                    if (timeSinceLastBrakeLog >= brakeLogInterval)
                    {
                        Debug.Log($"Brake still applied: {brake} (linearInput: {linearInput:F3})");
                        lastBrakeLogTime = Time.time;
                    }
                }
            }
            else
            {
                // 通常の加速/減速（モータートルクのスケーリングを改善）
                float targetMotor = maxMotorTorque * linearInput;
                
                // ブレーキが解除された直後の場合、モータートルクを段階的に増やす
                if (wasBraking)
                {
                    wasBraking = false;
                    brakeReleaseTime = Time.time;
                }
                
                // ブレーキ解除後の時間を計算（初期状態の場合はrampUpをスキップ）
                if (brakeReleaseTime < 0f)
                {
                    // 初期状態または初回の場合はrampUpをスキップ
                    motor = targetMotor;
                    brakeReleaseTime = Time.time; // 次回のために時刻を設定
                }
                else
                {
                    float timeSinceBrakeRelease = Time.time - brakeReleaseTime;
                    
                    if (timeSinceBrakeRelease < motorRampUpTime)
                    {
                        // ブレーキ解除後、指定時間内はモータートルクを段階的に増やす
                        float rampUpFactor = timeSinceBrakeRelease / motorRampUpTime;
                        motor = targetMotor * rampUpFactor;
                    }
                    else
                    {
                        // 通常のモータートルク
                        motor = targetMotor;
                    }
                }
                
                brake = 0f;
            }
            
            steering = maxSteeringAngle * angularInput;
            
            // 現在のsteering値を保存
            lastSteering = steering;
            lastMotor = motor;
            
            // デバッグログ（定期的に出力して問題を特定）
            if (Time.frameCount % 60 == 0) // 約1秒ごとに出力
            {
                Debug.Log($"ROS2 Control - Linear: {linearInput:F3}, Angular: {angularInput:F3}, Motor: {motor:F1}, Steering: {steering:F1}, UseROS2: {useROS2}");
            }
        }
        else if (fallbackToKeyboard)
        {
            // キーボード入力にフォールバック
            float verticalInput = Input.GetAxis("Vertical");
            if (!allowReverse)
            {
                verticalInput = Mathf.Max(0f, verticalInput);
            }
            if (Mathf.Abs(verticalInput) < linearInputDeadzone)
            {
                verticalInput = 0f;
            }
            targetForwardSpeed = verticalInput;
            steering = maxSteeringAngle * Input.GetAxis("Horizontal");
            brake = maxBrakeTorque * Input.GetAxis("Jump"); // スペースキーでブレーキ
            
            // キーボードでもブレーキ時はモータートルクを0に
            if (brake > 0f)
            {
                motor = 0f;
                holdStopped = true;
                // ブレーキが適用されたことを記録
                if (!wasBraking)
                {
                    wasBraking = true;
                }
            }
            else if (Mathf.Abs(verticalInput) <= 0f)
            {
                motor = 0f;
                brake = maxBrakeTorque;
                holdStopped = true;
                if (!wasBraking)
                {
                    wasBraking = true;
                }
            }
            else
            {
                // ブレーキが解除された直後の場合、モータートルクを段階的に増やす
                if (wasBraking)
                {
                    wasBraking = false;
                    brakeReleaseTime = Time.time;
                }
                
                float targetMotor = maxMotorTorque * verticalInput;
                
                // ブレーキ解除後の時間を計算（初期状態の場合はrampUpをスキップ）
                if (brakeReleaseTime < 0f)
                {
                    // 初期状態または初回の場合はrampUpをスキップ
                    motor = targetMotor;
                    brakeReleaseTime = Time.time; // 次回のために時刻を設定
                }
                else
                {
                    float timeSinceBrakeRelease = Time.time - brakeReleaseTime;
                    
                    if (timeSinceBrakeRelease < motorRampUpTime)
                    {
                        // ブレーキ解除後、指定時間内はモータートルクを段階的に増やす
                        float rampUpFactor = timeSinceBrakeRelease / motorRampUpTime;
                        motor = targetMotor * rampUpFactor;
                    }
                    else
                    {
                        // 通常のモータートルク
                        motor = targetMotor;
                    }
                }
            }
        }
        else
        {
            // ROS2がタイムアウトした場合、モータートルクとブレーキをリセット
            motor = 0f;
            brake = maxBrakeTorque; // タイムアウト時もブレーキを適用して停止（適切な値に修正）
            holdStopped = true;
            lastMotor = 0f;
            wasBraking = true; // タイムアウト時はブレーキが適用されている
            
            // タイムアウトログを定期的に出力（頻繁なログを防ぐ）
            float timeSinceLastLog = Time.time - lastTimeoutLogTime;
            if (timeSinceLastLog >= timeoutLogInterval)
            {
                float timeoutDuration = Time.time - lastROS2DataTime;
                if (ros2Available)
                {
                    Debug.LogWarning($"ROS2 timeout - no data received for {timeoutDuration:F2}s (ROS2 connection OK)");
                }
                else
                {
                    Debug.LogWarning($"ROS2 unavailable - connection lost or not initialized");
                }
                lastTimeoutLogTime = Time.time;
            }
        }
        
        float forwardSpeed = GetForwardSpeed();
        if (holdStopped)
        {
            motor = 0f;
            brake = maxBrakeTorque;
            ZeroForwardVelocityIfStopped(forwardSpeed);
        }
        else
        {
            float speedError = forwardSpeed - targetForwardSpeed;
            if (speedError > speedHoldTolerance)
            {
                motor = 0f;
                brake = Mathf.Max(brake, Mathf.Clamp(speedError * vehicleSpeedBrakeGain, 0f, maxBrakeTorque));
            }
        }
        
        
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                if(steering > 0){
                    axleInfo.leftFrontWheel.steerAngle = steering * 0.9f;
                    axleInfo.rightFrontWheel.steerAngle = steering * 1.2f;
                    axleInfo.leftBackWheel.steerAngle = -steering * 0.9f;
                    axleInfo.rightBackWheel.steerAngle = -steering * 1.2f;
                }else{
                    axleInfo.leftFrontWheel.steerAngle = steering * 1.2f;
                    axleInfo.rightFrontWheel.steerAngle = steering * 0.9f;
                    axleInfo.leftBackWheel.steerAngle = -steering * 1.2f;
                    axleInfo.rightBackWheel.steerAngle = -steering * 0.9f;
                }
            }
            if (axleInfo.motor) {
                bool brakeHandledByPID = false;
                if (usePIDControl)
                {
                    // PID制御を使用してホイールの回転速度を制御
                    ApplyPIDControl(axleInfo, motor, steering, brake, deltaTime);
                    brakeHandledByPID = true;
                }
                else
                {
                    // 従来の方法（PID制御なし）
                    // 各ホイールの回転数をチェックして制限を適用
                    float leftFrontRPM = axleInfo.leftFrontWheel.rpm;
                    float rightFrontRPM = axleInfo.rightFrontWheel.rpm;
                    float leftBackRPM = axleInfo.leftBackWheel.rpm;
                    float rightBackRPM = axleInfo.rightBackWheel.rpm;
                    
                    // 回転数に応じてモータートルクを調整
                    float leftFrontMotor = CalculateMotorTorque(motor, leftFrontRPM);
                    float rightFrontMotor = CalculateMotorTorque(motor, rightFrontRPM);
                    float leftBackMotor = CalculateMotorTorque(motor, leftBackRPM);
                    float rightBackMotor = CalculateMotorTorque(motor, rightBackRPM);
                    
                    axleInfo.leftFrontWheel.motorTorque = leftFrontMotor;
                    axleInfo.rightFrontWheel.motorTorque = rightFrontMotor;
                    axleInfo.leftBackWheel.motorTorque = leftBackMotor;
                    axleInfo.rightBackWheel.motorTorque = rightBackMotor;
                }

                if (!brakeHandledByPID)
                {
                    ApplyBrakeTorque(axleInfo, brake);
                }
            }
            else
            {
                ApplyBrakeTorque(axleInfo, brake);
            }

            // Rocker機構を更新（地形に合わせてロッカーアームを揺動させる）
            if (useRockerMechanism)
            {
                UpdateRockerMechanism(axleInfo, deltaTime);
            }

            // タイヤの見た目をステアリング軸に固定して更新
            UpdateWheelAssemblyVisual(axleInfo.leftFrontWheel, axleInfo.leftFrontWheelModel, axleInfo.leftFrontSteeringModel, steering);
            UpdateWheelAssemblyVisual(axleInfo.rightFrontWheel, axleInfo.rightFrontWheelModel, axleInfo.rightFrontSteeringModel, steering);
            UpdateWheelAssemblyVisual(axleInfo.leftBackWheel, axleInfo.leftBackWheelModel, axleInfo.leftBackSteeringModel, -steering);
            UpdateWheelAssemblyVisual(axleInfo.rightBackWheel, axleInfo.rightBackWheelModel, axleInfo.rightBackSteeringModel, -steering);
            // Debug.Log($"ROS2 Steering: {steering} , Motor: {motor}");
        }
    }
    
    // 現在の制御値を取得するメソッド（rover_control.csから移植、スレッドセーフ）
    public float GetLinearInput()
    {
        lock (velocityLock)
        {
            return linearVelocity;
        }
    }
    
    public float GetAngularInput()
    {
        lock (velocityLock)
        {
            return angularVelocity;
        }
    }
    
    // 制御データをリセット（rover_control.csから移植、スレッドセーフ）
    public void ResetControlData()
    {
        lock (velocityLock)
        {
            linearVelocity = 0f;
            angularVelocity = 0f;
        }
    }

    private float GetForwardSpeed()
    {
        if (carRigidbody == null) return 0f;

        return Vector3.Dot(carRigidbody.velocity, transform.forward);
    }

    private void ZeroForwardVelocityIfStopped(float forwardSpeed)
    {
        if (!clampForwardVelocityWhenStopped || carRigidbody == null)
            return;

        if (Mathf.Abs(forwardSpeed) > stoppedVelocityClampThreshold)
            return;

        carRigidbody.velocity -= transform.forward * forwardSpeed;
    }

    // PID制御を適用してホイールの回転速度を制御
    private void ApplyPIDControl(AxleInfo axleInfo, float baseMotor, float steering, float baseBrake, float deltaTime)
    {
        if (Mathf.Abs(maxMotorTorque) < 0.001f)
        {
            ApplyBrakeTorque(axleInfo, baseBrake);
            return;
        }

        // 目標速度を計算（m/s）
        // 線形速度から目標回転速度を計算
        float targetLinearSpeed = baseMotor / maxMotorTorque; // -1.0 ～ 1.0 の範囲
        
        // ステアリングによる左右の速度差を計算（差動駆動）
        float steeringDenominator = Mathf.Max(Mathf.Abs(maxSteeringAngle), 0.001f);
        float turnRadius = 1.0f / (Mathf.Max(Mathf.Abs(steering) / steeringDenominator, 0.01f)); // 旋回半径（仮想的）
        
        // 左右のホイールの目標速度を計算
        float trackWidth = 0.8f; // トレッド幅（仮想的、必要に応じて調整）
        
        // 左側と右側の目標速度を計算（差動駆動）
        float leftTargetSpeed, rightTargetSpeed;
        if (Mathf.Abs(steering) < 0.01f)
        {
            // 直進時
            leftTargetSpeed = targetLinearSpeed;
            rightTargetSpeed = targetLinearSpeed;
        }
        else
        {
            // 旋回時（差動駆動）
            float turnDirection = Mathf.Sign(steering);
            float innerRadius = turnRadius - trackWidth / 2f;
            float outerRadius = turnRadius + trackWidth / 2f;
            
            if (turnDirection > 0)
            {
                // 右旋回：左側が外側
                leftTargetSpeed = targetLinearSpeed * (outerRadius / turnRadius);
                rightTargetSpeed = targetLinearSpeed * (innerRadius / turnRadius);
            }
            else
            {
                // 左旋回：右側が外側
                leftTargetSpeed = targetLinearSpeed * (innerRadius / turnRadius);
                rightTargetSpeed = targetLinearSpeed * (outerRadius / turnRadius);
            }
        }
        
        // 目標速度をRPMに変換（m/s → RPM）
        // RPM = (速度 m/s) / (2 * π * 半径 m) * 60
        // 目標速度は-1.0～1.0の範囲なので、maxRPMを掛けて実際のRPMに変換
        float leftTargetRPM = leftTargetSpeed * maxRPM;
        float rightTargetRPM = rightTargetSpeed * maxRPM;
        
        // 各ホイールにPID制御を適用
        ApplyPIDToWheel(axleInfo.leftFrontWheel, leftTargetRPM, baseBrake, deltaTime);
        ApplyPIDToWheel(axleInfo.leftBackWheel, leftTargetRPM, baseBrake, deltaTime);
        ApplyPIDToWheel(axleInfo.rightFrontWheel, rightTargetRPM, baseBrake, deltaTime);
        ApplyPIDToWheel(axleInfo.rightBackWheel, rightTargetRPM, baseBrake, deltaTime);
    }
    
    // 個別のホイールにPID制御を適用
    private void ApplyPIDToWheel(WheelCollider wheel, float targetRPM, float baseBrake, float deltaTime)
    {
        if (wheel == null || !wheelPIDControllers.ContainsKey(wheel))
            return;
        
        PIDController pid = wheelPIDControllers[wheel];

        if (baseBrake > 0f)
        {
            pid.Reset();
            wheel.motorTorque = 0f;
            wheel.brakeTorque = baseBrake;
            return;
        }

        float currentRPM = wheel.rpm;
        
        // PID制御でモータートルクを計算
        float pidOutput = pid.Calculate(targetRPM, currentRPM, deltaTime);
        
        // PID出力をモータートルクに変換（制限を適用）
        float motorTorque = 0f;
        float brakeTorque = 0f;

        if (allowReverse && targetRPM < -0.01f)
        {
            if (pidOutput < 0f)
            {
                motorTorque = Mathf.Clamp(pidOutput, -maxMotorTorque, 0f);
            }
            else
            {
                brakeTorque = Mathf.Clamp(pidOutput * speedHoldBrakeMultiplier, 0f, maxBrakeTorque);
            }
        }
        else
        {
            if (pidOutput > 0f)
            {
                motorTorque = Mathf.Clamp(pidOutput, 0f, maxMotorTorque);
            }
            else
            {
                brakeTorque = Mathf.Clamp(-pidOutput * speedHoldBrakeMultiplier, 0f, maxBrakeTorque);
            }
        }
        
        // 回転数制限を適用（安全のため）
        motorTorque = CalculateMotorTorque(motorTorque, currentRPM);
        if (!allowReverse)
        {
            motorTorque = Mathf.Max(0f, motorTorque);
        }
        
        wheel.motorTorque = motorTorque;
        wheel.brakeTorque = brakeTorque;
    }

    private void ApplyBrakeTorque(AxleInfo axleInfo, float brake)
    {
        if (axleInfo.leftFrontWheel != null)
            axleInfo.leftFrontWheel.brakeTorque = brake;
        if (axleInfo.rightFrontWheel != null)
            axleInfo.rightFrontWheel.brakeTorque = brake;
        if (axleInfo.leftBackWheel != null)
            axleInfo.leftBackWheel.brakeTorque = brake;
        if (axleInfo.rightBackWheel != null)
            axleInfo.rightBackWheel.brakeTorque = brake;
    }
    
    // 回転数に応じてモータートルクを計算する関数
    private float CalculateMotorTorque(float baseMotor, float rpm) {
        if (!allowReverse)
        {
            baseMotor = Mathf.Max(0f, baseMotor);
        }

        float absRPM = Mathf.Abs(rpm);
        
        // 後退時（負のモータートルク）は回転数制限を緩くする
        float currentMaxRPM = maxRPM;
        float currentRpmLimitStart = rpmLimitStart;
        
        if (baseMotor < 0) {
            currentMaxRPM *= reverseRPMMultiplier;
            currentRpmLimitStart *= reverseRPMMultiplier;
        }
        
        // 最大回転数を超えている場合は0
        if (absRPM >= currentMaxRPM) {
            return 0f;
        }
        
        // 回転数制限開始点を超えている場合は徐々に減らす
        // rpmLimitStartとmaxRPMが同じ値の場合、制限を無効化
        if (currentRpmLimitStart < currentMaxRPM && absRPM > currentRpmLimitStart) {
            float t = (absRPM - currentRpmLimitStart) / (currentMaxRPM - currentRpmLimitStart);
            float result = baseMotor * (1f - t);
            return result;
        }
        
        // 通常のモータートルク
        return baseMotor;
    }

    // タイヤの見た目をコライダーに合わせて動かす関数
    private void UpdateWheelVisual(WheelCollider collider, Transform model) {
        if (collider == null || model == null) return;

        Rigidbody modelRigidbody = model.GetComponent<Rigidbody>();
        if (modelRigidbody != null && !modelRigidbody.isKinematic)
        {
            Debug.LogWarning($"Skipping wheel visual update for {model.name}: it has a non-kinematic Rigidbody.");
            return;
        }

        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        model.position = pos;
        model.rotation = rot;
    }

    private void UpdateWheelAssemblyVisual(WheelCollider collider, Transform wheelModel, Transform steeringModel, float steeringAngle)
    {
        if (!keepWheelModelsOnSteeringAxis || steeringModel == null)
        {
            UpdateWheelVisual(collider, wheelModel);
            UpdateSteeringModel(steeringModel, steeringAngle);
            return;
        }

        if (collider == null) return;

        CacheVisualTransform(steeringModel);
        CacheVisualTransform(wheelModel);

        Vector3 wheelPosition;
        Quaternion wheelRotation;
        collider.GetWorldPose(out wheelPosition, out wheelRotation);

        steeringModel.localPosition = visualInitialLocalPositions[steeringModel];
        steeringModel.localRotation = visualInitialLocalRotations[steeringModel] * Quaternion.Euler(0f, steeringAngle, 0f);

        if (wheelModel == null) return;

        Rigidbody modelRigidbody = wheelModel.GetComponent<Rigidbody>();
        if (modelRigidbody != null && !modelRigidbody.isKinematic)
        {
            Debug.LogWarning($"Skipping wheel visual update for {wheelModel.name}: it has a non-kinematic Rigidbody.");
            return;
        }

        if (parentWheelModelsToSteeringAxis && wheelModel.parent != steeringModel)
        {
            wheelModel.SetParent(steeringModel, true);
            CacheVisualTransform(wheelModel, true);
        }

        if (wheelModel.parent == steeringModel)
        {
            wheelModel.localPosition = visualInitialLocalPositions[wheelModel];
            wheelModel.localRotation = Quaternion.Inverse(steeringModel.rotation) * wheelRotation;
        }
        else
        {
            wheelModel.position = wheelPosition;
            wheelModel.rotation = wheelRotation;
        }
    }

    private Vector3 ToParentLocalVector(Transform transformToMove, Vector3 worldVector)
    {
        if (transformToMove.parent == null)
        {
            return worldVector;
        }

        return transformToMove.parent.InverseTransformVector(worldVector);
    }

    private void InitializeWheelVisuals()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            InitializeWheelVisual(axleInfo.leftFrontWheelModel, axleInfo.leftFrontSteeringModel, axleInfo.leftRockerPivot);
            InitializeWheelVisual(axleInfo.rightFrontWheelModel, axleInfo.rightFrontSteeringModel, axleInfo.rightRockerPivot);
            InitializeWheelVisual(axleInfo.leftBackWheelModel, axleInfo.leftBackSteeringModel, axleInfo.leftRockerPivot);
            InitializeWheelVisual(axleInfo.rightBackWheelModel, axleInfo.rightBackSteeringModel, axleInfo.rightRockerPivot);
        }
    }

    private void InitializeWheelVisual(Transform wheelModel, Transform steeringModel, Transform rockerPivot)
    {
        if (parentSteeringModelsToRocker && steeringModel != null && rockerPivot != null && steeringModel.parent != rockerPivot)
        {
            steeringModel.SetParent(rockerPivot, true);
        }

        CacheVisualTransform(steeringModel);

        if (wheelModel == null) return;

        if (parentWheelModelsToSteeringAxis && steeringModel != null && wheelModel.parent != steeringModel)
        {
            wheelModel.SetParent(steeringModel, true);
        }

        CacheVisualTransform(wheelModel, true);
    }

    private void CacheVisualTransform(Transform visual, bool overwrite = false)
    {
        if (visual == null) return;

        if (overwrite || !visualInitialLocalPositions.ContainsKey(visual))
        {
            visualInitialLocalPositions[visual] = visual.localPosition;
            visualInitialLocalRotations[visual] = visual.localRotation;
        }
    }

    private void SanitizeWheelVisualPhysics()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            SanitizeVisualTransform(axleInfo.leftFrontWheelModel);
            SanitizeVisualTransform(axleInfo.rightFrontWheelModel);
            SanitizeVisualTransform(axleInfo.leftBackWheelModel);
            SanitizeVisualTransform(axleInfo.rightBackWheelModel);
        }
    }

    private void SanitizeVisualTransform(Transform visual)
    {
        if (visual == null) return;

        Rigidbody[] rigidbodies = visual.GetComponentsInChildren<Rigidbody>(true);
        foreach (Rigidbody rb in rigidbodies)
        {
            rb.isKinematic = true;
            rb.useGravity = false;
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        Collider[] colliders = visual.GetComponentsInChildren<Collider>(true);
        foreach (Collider visualCollider in colliders)
        {
            if (visualCollider is WheelCollider) continue;
            visualCollider.enabled = false;
        }
    }
    
    // ステアリング角度と同じ角度で別の3Dモデルを回転させる関数
    private void UpdateSteeringModel(Transform steeringModel, float steeringAngle) {
        if (steeringModel == null) return;
        
        // Y軸周りの回転を適用（ステアリング角度）
        steeringModel.localRotation = Quaternion.Euler(0f, steeringAngle, 0f);
    }
    
    // ================== Rocker機構 ==================
    // 火星ローバーのようなロッカーサスペンション：
    // 左右それぞれのロッカーアーム（前輪と後輪の親Transform）が
    // 地形の凹凸に合わせてピッチ軸周りに揺動し、4輪の接地を維持する。
    // 差動バーを有効にすると左右の揺動の平均成分が打ち消され、
    // 車体は左右ロッカーの中間角度を保つ（実機の差動リンクの近似）。
    
    // Rocker機構の更新（FixedUpdateから呼び出し）
    private void UpdateRockerMechanism(AxleInfo axleInfo, float deltaTime)
    {
        if (deltaTime <= 0f) deltaTime = Time.fixedDeltaTime;
        
        float leftTarget = 0f;
        float rightTarget = 0f;
        bool hasLeft = axleInfo.leftRockerPivot != null;
        bool hasRight = axleInfo.rightRockerPivot != null;
        
        // 左側：前輪と後輪の接地高さの差からロッカーの目標角度を計算
        if (hasLeft)
        {
            leftTarget = CalculateRockerTargetAngle(axleInfo.leftFrontWheel, axleInfo.leftBackWheel);
        }
        // 右側も同様
        if (hasRight)
        {
            rightTarget = CalculateRockerTargetAngle(axleInfo.rightFrontWheel, axleInfo.rightBackWheel);
        }
        
        // 差動バー：左右の差分だけを使い、片側が上がると反対側が下がるように連動させる
        if (useDifferentialBar && hasLeft && hasRight)
        {
            float linkedTarget = (leftTarget - rightTarget) * 0.5f;
            leftTarget = linkedTarget;
            rightTarget = -linkedTarget;
        }
        
        // ゲインと最大角度制限を適用
        leftTarget = Mathf.Clamp(leftTarget * rockerGain, -maxRockerAngle, maxRockerAngle);
        rightTarget = Mathf.Clamp(rightTarget * rockerGain, -maxRockerAngle, maxRockerAngle);
        
        // 滑らかに追従させてピボットに適用
        if (hasLeft)
            ApplyRockerRotation(axleInfo.leftRockerPivot, leftTarget, deltaTime);
        if (hasRight)
            ApplyRockerRotation(axleInfo.rightRockerPivot, rightTarget, deltaTime);
    }
    
    // 前輪と後輪の接地点の高さ差からロッカーの目標角度（度）を計算
    private float CalculateRockerTargetAngle(WheelCollider frontWheel, WheelCollider rearWheel)
    {
        if (frontWheel == null || rearWheel == null) return 0f;
        
        // 車体ローカル座標での接地高さを取得
        float frontY = GetWheelGroundLocalY(frontWheel);
        float rearY = GetWheelGroundLocalY(rearWheel);
        
        // 前後輪の車体ローカルZ方向距離（ロッカーアームの長さに相当）
        float frontZ = transform.InverseTransformPoint(frontWheel.transform.position).z;
        float rearZ = transform.InverseTransformPoint(rearWheel.transform.position).z;
        float armLength = Mathf.Abs(frontZ - rearZ);
        if (armLength < 0.001f) return 0f;
        
        // 前が高い（前輪が障害物に乗り上げた）ときに前が持ち上がる方向へ回転
        // Unityの左手系でX軸正回転はノーズダウンなので符号を反転
        return -Mathf.Atan2(frontY - rearY, armLength) * Mathf.Rad2Deg;
    }
    
    // ホイールの接地点の車体ローカルY座標を取得（ホイール中心高さ換算）
    private float GetWheelGroundLocalY(WheelCollider wheel)
    {
        WheelHit hit;
        if (wheel.GetGroundHit(out hit))
        {
            // 接地している場合：接地点＋半径 ＝ ホイール中心の実効高さ
            Vector3 localHit = transform.InverseTransformPoint(hit.point);
            return localHit.y + wheel.radius;
        }
        
        // 接地していない場合：サスペンションが最大に伸びた位置とみなす
        Vector3 localWheelPos = transform.InverseTransformPoint(wheel.transform.position);
        return localWheelPos.y - wheel.suspensionDistance;
    }
    
    // ロッカーピボットに滑らかに回転を適用
    private void ApplyRockerRotation(Transform pivot, float targetAngle, float deltaTime)
    {
        // 未登録のピボット（実行中に割り当てられた場合）を初期化
        if (!rockerInitialRotations.ContainsKey(pivot))
        {
            rockerInitialRotations[pivot] = pivot.localRotation;
            rockerCurrentAngles[pivot] = 0f;
        }
        
        // 現在角度から目標角度へ滑らかに補間（急激な揺れを防止）
        float current = rockerCurrentAngles[pivot];
        float smoothed = Mathf.Lerp(current, targetAngle, 1f - Mathf.Exp(-rockerSmoothSpeed * deltaTime));
        rockerCurrentAngles[pivot] = smoothed;
        
        // 初期回転を基準にピッチ軸（ローカルX軸）周りの回転を適用
        // ピボットの子であるWheelColliderも一緒に動き、物理挙動に反映される
        pivot.localRotation = rockerInitialRotations[pivot] * Quaternion.Euler(smoothed, 0f, 0f);
    }
}
[System.Serializable]
public class AxleInfo {
    public WheelCollider leftFrontWheel;
    public WheelCollider rightFrontWheel;
    public WheelCollider leftBackWheel;
    public WheelCollider rightBackWheel;
    public Transform leftFrontWheelModel;
    public Transform rightFrontWheelModel;
    public Transform leftBackWheelModel;
    public Transform rightBackWheelModel;
    public Transform leftFrontSteeringModel; // 左前ステアリングモデル
    public Transform rightFrontSteeringModel; // 右前ステアリングモデル
    public Transform leftBackSteeringModel; // 左後ステアリングモデル
    public Transform rightBackSteeringModel; // 右後ステアリングモデル
    public Transform leftRockerPivot; // 左ロッカーアームのピボット（左前輪・左後輪のWheelColliderの親）
    public Transform rightRockerPivot; // 右ロッカーアームのピボット（右前輪・右後輪のWheelColliderの親）
    public bool motor; //駆動輪か?
    public bool steering; //ハンドル操作をしたときに角度が変わるか？
}
