using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("Car Specs")]
    public float wheelBase = 0.8f;
    public float maxSteerAngle = 30f;
    public float engineForce = 10f;
    public float brakeForce = 15f;
    public float mass = 1f;
    public float dragCoeff = 2f;     // 抵抗大きめ
    public float maxSpeed = 5f;      // 最高速度制限

    [Header("LiDAR")]
    public float lidarFOV = 120f;
    public int lidarRays = 31;
    public float lidarMaxDist = 20f;

    [Header("Network")]
    public string host = "127.0.0.1";
    public int port = 5000;

    Rigidbody rb;
    TcpClient client;
    NetworkStream stream;

    float velocity = 0f;
    float steerAngle = 0f;

    float[] lidarDistances;
    int carMask;

    [Serializable]
    class StatePacket {
        public float x, y, z;
        public float yaw;
        public float[] lidar;
    }

    [Serializable]
    class CommandPacket {
        public float throttle;
        public float steering;
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        rb.mass = mass;

        // ★ 壁すり抜け対策（最重要）★
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        lidarDistances = new float[lidarRays];
        carMask = ~(1 << LayerMask.NameToLayer("Car"));

        // 接続（タイムアウト回避の非同期接続）
        StartCoroutine(TryConnect());
    }

    // ----- 非同期接続（Unityフリーズ防止） -----
    System.Collections.IEnumerator TryConnect()
    {
        Debug.Log("Connecting to Python...");

        var cli = new TcpClient();
        var task = cli.ConnectAsync(host, port);

        float timeout = 2f;
        float t = 0f;

        while (!task.IsCompleted)
        {
            if (t > timeout)
            {
                Debug.LogWarning("Python server not found. Running without connection.");
                yield break;
            }

            t += Time.deltaTime;
            yield return null;
        }

        if (cli.Connected)
        {
            Debug.Log("Connected to Python!");
            client = cli;
            stream = cli.GetStream();
        }
        else
        {
            Debug.LogWarning("Connection failed.");
        }
    }

    void FixedUpdate()
    {
        if (stream == null) return;

        UpdateLidar();
        SendState();

        CommandPacket cmd = ReceiveCommand();
        if (cmd != null)
            ApplyControl(cmd);
    }

    // ------------------- LiDAR -------------------

    void UpdateLidar()
    {
        Vector3 origin = transform.position;

        for (int i = 0; i < lidarRays; i++)
        {
            float t = (float)i / (lidarRays - 1);
            float angle = -lidarFOV / 2 + lidarFOV * t;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, lidarMaxDist, carMask))
                lidarDistances[i] = hit.distance;
            else
                lidarDistances[i] = lidarMaxDist;
        }
    }

    // ------------------- Network -------------------

    void SendState()
    {
        StatePacket state = new StatePacket
        {
            x = transform.position.x,
            y = transform.position.y,
            z = transform.position.z,
            yaw = transform.eulerAngles.y * Mathf.Deg2Rad,
            lidar = lidarDistances
        };

        string json = JsonUtility.ToJson(state) + "\n";
        byte[] buf = Encoding.UTF8.GetBytes(json);

        try { stream.Write(buf, 0, buf.Length); } catch {}
    }

    CommandPacket ReceiveCommand()
    {
        try
        {
            string line = ReadLine(stream);
            if (string.IsNullOrEmpty(line)) return null;
            return JsonUtility.FromJson<CommandPacket>(line);
        }
        catch { return null; }
    }

    string ReadLine(NetworkStream s)
    {
        StringBuilder sb = new();
        int b;

        while ((b = s.ReadByte()) != -1)
        {
            if (b == '\n') break;
            sb.Append((char)b);
        }

        return sb.ToString();
    }

    // ------------------- Physics（改良版） -------------------

    void ApplyControl(CommandPacket cmd)
    {
        float dt = Time.fixedDeltaTime;

        // throttle
        float throttle = Mathf.Clamp(cmd.throttle, -1f, 1f);
        float accelCmd = (throttle >= 0f)
            ? throttle * engineForce
            : throttle * brakeForce;

        float accel = (accelCmd / mass) - dragCoeff * velocity;
        velocity += accel * dt;

        // ★ 最高速度の物理的制限 ★
        velocity = Mathf.Clamp(velocity, 0, maxSpeed);

        // steering
        steerAngle = Mathf.Clamp(cmd.steering, -1f, 1f) * maxSteerAngle;

        float turnRate = Mathf.Tan(steerAngle * Mathf.Deg2Rad) / wheelBase;
        float yawDelta = turnRate * velocity * dt;

        Quaternion newRot = rb.rotation * Quaternion.Euler(0, yawDelta * Mathf.Rad2Deg, 0);

        // ★ rb.velocity を直接セット（MovePosition の壁抜け対策）★
        rb.velocity = newRot * Vector3.forward * velocity;

        rb.MoveRotation(newRot);
    }
}
