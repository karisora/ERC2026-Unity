using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Unity.Collections;
using System.Threading.Tasks;
using System;
using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;

[RequireComponent(typeof(ROS2UnityComponent))]
public class RealSense : MonoBehaviour
{
    [SerializeField] private int width = 640;
    [SerializeField] private int height = 360;
    [SerializeField] private int fps = 15;
    [SerializeField] private float vFov = 60.0f;
    [SerializeField] private string frameID = "realsense_link";
    [SerializeField] private string imageTopic = "/realsense/image_raw";
    [SerializeField] private string infoTopic = "/realsense/camera_info";
    [SerializeField] private string pointCloudTopic = "/realsense/cloud";

    [Header("PointCloud settings")]
    [Tooltip("Stride for sampling pixels when building the point cloud (larger = less points, faster)")]
    [Range(1, 32)]
    public int pointCloudStride = 13;
    [Tooltip("Maximum number of points to publish (safety cap)")]
    public int maxPoints = 20000;

    [SerializeField] private ComputeShader imageProcessingShader; // ComputeShader を追加（Inspectorで割当て）

    // キャッシュするカーネルインデックス（無効時は -1）
    private int imageKernel = -1;

    private Camera cam;
    private RenderTexture rt;
    private float lastPublishTime = 0f;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.Image> imagePub;
    private IPublisher<sensor_msgs.msg.CameraInfo> infoPub;
    private IPublisher<sensor_msgs.msg.PointCloud2> pc2Pub;
    // GPU から読み取ったピクセル（bottom-left origin）
    private Color32[] gpuPixels = null;

    // for intrinsics
    private Matrix4x4 rosOpticalToPixel;

    void OnEnable()
    {
        cam = GetComponent<Camera>();
    }

    void Start()
    {
        cam = GetComponent<Camera>();
        cam.enabled = true;
        cam.depthTextureMode = DepthTextureMode.None;
        cam.fieldOfView = vFov;
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane = 3000f;

        // create and assign RenderTexture (ensure exact size)
        rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        rt.Create();
        cam.targetTexture = rt;

        // compute intrinsics helper
        Matrix4x4 rosOpticalToRosView = Matrix4x4.identity;
        rosOpticalToRosView.SetColumn(0, new Vector4(0, -1, 0, 0));
        rosOpticalToRosView.SetColumn(1, new Vector4(0, 0, -1, 0));
        rosOpticalToRosView.SetColumn(2, new Vector4(1, 0, 0, 0));

        Matrix4x4 rosViewToGlView = Matrix4x4.identity;
        rosViewToGlView.SetColumn(0, new Vector4(0, 0, -1, 0));
        rosViewToGlView.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosViewToGlView.SetColumn(2, new Vector4(0, 1, 0, 0));

        Matrix4x4 glViewToGlNdc = cam.nonJitteredProjectionMatrix;
        Matrix4x4 glNDCToGlUv = Matrix4x4.identity;
        glNDCToGlUv.SetRow(0, new Vector4(0.5f, 0, 0, 0.5f));
        glNDCToGlUv.SetRow(1, new Vector4(0, 0.5f, 0, 0.5f));
        glNDCToGlUv.SetRow(2, new Vector4(0, 0, 0.5f, 0.5f));

        Matrix4x4 glUvToPixel = Matrix4x4.identity;
        glUvToPixel.SetRow(0, new Vector4(width, 0, 0, 0));
        glUvToPixel.SetRow(1, new Vector4(0, -height, 0, height));

        rosOpticalToPixel = glUvToPixel * glNDCToGlUv * glViewToGlNdc * rosViewToGlView * rosOpticalToRosView;

        ros2Unity = GetComponent<ROS2UnityComponent>();

        // ComputeShader のカーネルを一度だけ取得（無ければ -1 を保持）
        if (imageProcessingShader != null)
        {
            try
            {
                imageKernel = imageProcessingShader.FindKernel("CSMain");
            }
            catch (Exception)
            {
                imageKernel = -1;
                // 静かにフォールバック（Inspectorで割当てやシェーダの名前を確認してください）
            }
        }
    }

    void OnDestroy()
    {
        if (rt != null)
        {
            cam.targetTexture = null;
            rt.Release();
            Destroy(rt);
            rt = null;
        }
    }

    void Update()
    {
        if (ros2Unity == null || !ros2Unity.Ok()) return;

        // lazy-init node and publishers
        if (ros2Node == null)
        {
            string uniqueNodeName = "realsense_" + gameObject.name;
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            imagePub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(imageTopic);
            infoPub = ros2Node.CreatePublisher<sensor_msgs.msg.CameraInfo>(infoTopic);
            pc2Pub = ros2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pointCloudTopic);
        }

        // throttle by fps
        if (Time.time - lastPublishTime < 1.0f / Mathf.Max(1, fps)) return;
        lastPublishTime = Time.time;

        if (rt == null) return;

        // Ensure camera renders into the RenderTexture this frame
        var prevActive = RenderTexture.active;
        RenderTexture.active = rt;
        cam.targetTexture = rt;
        cam.Render(); // <-- render to RT
        RenderTexture.active = prevActive;

        // GPUで処理：ComputeShaderがあればSource->Resultへ、なければ単純コピー（Blit）
        RenderTexture processedTexture = new RenderTexture(rt.width, rt.height, 0, RenderTextureFormat.ARGB32);
        processedTexture.enableRandomWrite = true;
        processedTexture.Create();

        if (imageProcessingShader != null && imageKernel >= 0)
        {
            imageProcessingShader.SetTexture(imageKernel, "Source", rt);
            imageProcessingShader.SetTexture(imageKernel, "Result", processedTexture);
            imageProcessingShader.SetInt("Width", rt.width);
            imageProcessingShader.SetInt("Height", rt.height);
            int tgx = Mathf.CeilToInt(rt.width / 8.0f);
            int tgy = Mathf.CeilToInt(rt.height / 8.0f);
            imageProcessingShader.Dispatch(imageKernel, tgx, tgy, 1);
        }
        else
        {
            // ComputeShader が未設定、またはカーネルが見つからない場合はフォールバックでコピー
            Graphics.Blit(rt, processedTexture);
        }

        // 非同期読み出し（同期で待機）：フォーマットを明示して安定したチャネル順で取得
        var req = AsyncGPUReadback.Request(processedTexture, 0, TextureFormat.RGBA32);
        req.WaitForCompletion();
        if (req.hasError)
        {
            processedTexture.Release();
            Destroy(processedTexture);
            return;
        }
        var native = req.GetData<Color32>();
        if (gpuPixels == null || gpuPixels.Length != native.Length) gpuPixels = new Color32[native.Length];
        native.CopyTo(gpuPixels);

        // Build sensor_msgs/Image (rgb8)
        var imgMsg = new sensor_msgs.msg.Image();
        imgMsg.Header = new std_msgs.msg.Header();
        imgMsg.Header.Frame_id = frameID;
        ros2Node.clock.UpdateROSClockTime(imgMsg.Header.Stamp);

        imgMsg.Height = (uint)rt.height;
        imgMsg.Width = (uint)rt.width;
        imgMsg.Encoding = "rgb8";
        imgMsg.Is_bigendian = 0;
        imgMsg.Step = (uint)(rt.width * 3);

        // GPU読み出し結果(gpuPixels)は bottom-left origin の並びなので、
        // ROS/rviz用に上下反転してバイト配列に詰める
        int w = rt.width;
        int h = rt.height;
        byte[] data = new byte[w * h * 3];
        for (int y = 0; y < h; ++y)
        {
            int srcY = h - 1 - y; // flip
            for (int x = 0; x < w; ++x)
            {
                int srcIdx = srcY * w + x;
                int dstIdx = (y * w + x) * 3;
                Color32 c = gpuPixels[srcIdx];
                // AsyncGPUReadback に RGBA32 を指定しているので Color32 は (r,g,b,a) の順で安定しているはず
                data[dstIdx + 0] = c.r; // R
                data[dstIdx + 1] = c.g; // G
                data[dstIdx + 2] = c.b; // B
            }
        }
        imgMsg.Data = data;
        imagePub.Publish(imgMsg);

        // Build CameraInfo message and fill using rosOpticalToPixel
        var infoMsg = new sensor_msgs.msg.CameraInfo();
        infoMsg.Header = new std_msgs.msg.Header();
        infoMsg.Header.Frame_id = frameID;
        ros2Node.clock.UpdateROSClockTime(infoMsg.Header.Stamp);

        infoMsg.Height = (uint)rt.height;
        infoMsg.Width = (uint)rt.width;
        infoMsg.Distortion_model = "";
        infoMsg.D = new double[0];

        double fx = rosOpticalToPixel[0, 0];
        double fy = rosOpticalToPixel[1, 1];
        double cx = rosOpticalToPixel[0, 2];
        double cy = rosOpticalToPixel[1, 2];

        // K: row-major 3x3
        for (int i = 0; i < 9; ++i) infoMsg.K[i] = 0.0;
        infoMsg.K[0] = fx; infoMsg.K[2] = cx;
        infoMsg.K[4] = fy; infoMsg.K[5] = cy;
        infoMsg.K[8] = 1.0;

        // R = identity
        for (int i = 0; i < 9; ++i) infoMsg.R[i] = 0.0;
        infoMsg.R[0] = 1.0; infoMsg.R[4] = 1.0; infoMsg.R[8] = 1.0;

        // P (3x4)
        for (int i = 0; i < 12; ++i) infoMsg.P[i] = 0.0;
        infoMsg.P[0] = fx; infoMsg.P[2] = cx; infoMsg.P[5] = fy; infoMsg.P[6] = cy; infoMsg.P[10] = 1.0;

        infoPub.Publish(infoMsg);

        processedTexture.Release();
        Destroy(processedTexture);

        // Build PointCloud2 by raycasting (sampled by stride)
        List<Vector3> points = new List<Vector3>();
        // also store packed rgb as float (ROS typical representation)
        List<float> packedColors = new List<float>();
         int stride = Mathf.Max(1, pointCloudStride);
         for (int y = 0; y < rt.height; y += stride)
         {
            for (int x = 0; x < rt.width; x += stride)
            {
                if (points.Count >= maxPoints) break;

            // use viewport coords and ViewportPointToRay (correct for RenderTexture)
            float vx = (x + 0.5f) / (float)rt.width;
            float vy = (y + 0.5f) / (float)rt.height;
            Ray ray = cam.ViewportPointToRay(new Vector3(vx, vy, 0f));

            if (Physics.Raycast(ray, out RaycastHit hit, cam.farClipPlane))
            {
                // Convert hit point into camera-local coordinates, then to ROS coordinates
                // (Transformations.Unity2Ros extension is used here)
                Vector3 localPoint = cam.transform.InverseTransformPoint(hit.point);
                Vector3 rosPoint = localPoint.Unity2Ros();
                points.Add(rosPoint);
                // also store packed rgb as float (ROS typical representation)
                // gpuPixels は bottom-left origin の並び（レンダリング座標）なのでそのまま参照
                Color32 col = gpuPixels[y * rt.width + x];
                // RGBA32 指定なので r,g,b をそのまま使う
                byte r = col.r;
                byte g = col.g;
                byte b = col.b;
                uint rgb = ((uint)r << 16) | ((uint)g << 8) | (uint)b;
                // Pack RGB as float (ROS typical representation)
                float rgbF = BitConverter.ToSingle(BitConverter.GetBytes(rgb), 0);
                packedColors.Add(rgbF);
                }
            }
            if (points.Count >= maxPoints) break;
         }

         // Prepare PointCloud2 message
         var pc2 = new sensor_msgs.msg.PointCloud2();
         pc2.Header = new std_msgs.msg.Header();
         pc2.Header.Frame_id = frameID;
         ros2Node.clock.UpdateROSClockTime(pc2.Header.Stamp);

         int pointCount = points.Count;
         pc2.Height = 1; // unorganized
         pc2.Width = (uint)pointCount;
         pc2.Is_bigendian = false;
        // RGB を含める -> 4 要素 (x,y,z,rgb) 各 4 バイトの float32
        pc2.Is_dense = true;
        int pointStep = 4 * 4; // x,y,z,rgb
        pc2.Point_step = (uint)pointStep;
        pc2.Row_step = (uint)(pointStep * pointCount);
        // fields: x,y,z,rgb (rgb は float32 として格納されるビットパターン)
        pc2.Fields = new sensor_msgs.msg.PointField[4];
        pc2.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 }; // FLOAT32=7
        pc2.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 };
        pc2.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 };
        pc2.Fields[3] = new sensor_msgs.msg.PointField { Name = "rgb", Offset = 12, Datatype = 7, Count = 1 };

        byte[] pcdata = new byte[pointCount * pointStep];
        for (int i = 0; i < pointCount; ++i)
        {
            int baseIdx = i * pointStep;
            var p = points[i]; // now already in ROS camera frame (x,y,z)
            Buffer.BlockCopy(BitConverter.GetBytes(p.x), 0, pcdata, baseIdx + 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.y), 0, pcdata, baseIdx + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.z), 0, pcdata, baseIdx + 8, 4);
            // 対応するカラー（存在しない場合は黒）
            float rgbF = (i < packedColors.Count) ? packedColors[i] : BitConverter.ToSingle(BitConverter.GetBytes(0u), 0);
            Buffer.BlockCopy(BitConverter.GetBytes(rgbF), 0, pcdata, baseIdx + 12, 4);
        }
        pc2.Data = pcdata;

        pc2Pub.Publish(pc2);
    }
}
