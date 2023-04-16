using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System;
using System.Text.RegularExpressions;

public class MessageParser
{
    public static float[] ParseMessage(string message)
    {
        float[] values = new float[12];
        string pattern = @"([a-z]{2})(\d+)"; // regex pattern to match two letters followed by a number
        MatchCollection matches = Regex.Matches(message, pattern);

        foreach (Match match in matches)
        {
            string code = match.Groups[1].Value;
            float value = float.Parse(match.Groups[2].Value);

            switch (code)
            {
                case "rx":
                    values[0] = value;
                    break;
                case "ry":
                    values[1] = value;
                    break;
                case "rz":
                    values[2] = value;
                    break;
                case "yx":
                    values[3] = value;
                    break;
                case "yy":
                    values[4] = value;
                    break;
                case "yz":
                    values[5] = value;
                    break;
                case "bx":
                    values[6] = value;
                    break;
                case "by":
                    values[7] = value;
                    break;
                case "bz":
                    values[8] = value;
                    break;
                case "gx":
                    values[9] = value;
                    break;
                case "gy":
                    values[10] = value;
                    break;
                case "gz":
                    values[11] = value;
                    break;
                default:
                    // ignore unknown codes
                    break;
            }
        }

        return values;
    }
}

public class CubeController : MonoBehaviour
{
    public Transform redCube;
    public Transform yellowCube;
    public Transform blueCube;
    public Transform greenCube;

    private TcpClient _client;
    private NetworkStream _stream;

    private bool[] _heldCubes = new bool[4];
    private bool[] _sentCubes = new bool[4];

    private const int Port = 13322;
    private const string IpAddress = "198.168.1.8";

    private void Start()
    {
        // connect to server
        _client = new TcpClient();
        _client.Connect(IPAddress.Parse(IpAddress), Port);
        _stream = _client.GetStream();
    }

    private void Start()
    {
        // read data from stream
        if (_stream.DataAvailable)
        {
            byte[] data = new byte[1024];
            int bytesRead = _stream.Read(data, 0, data.Length);
            string rawMessage = Encoding.ASCII.GetString(data, 0, bytesRead);
            float[] values = MessageParser.ParseMessage(rawMessage);

            // set cube positions
            redCube.transform.position = new Vector3(values[0], values[1], values[2]);
            yellowCube.transform.position = new Vector3(values[3], values[4], values[5]);
            blueCube.transform.position = new Vector3(values[6], values[7], values[8]);
            greenCube.transform.position = new Vector3(values[9], values[10], values[11]);

            Debug.Log($"Received values: [{string.Join(", ", values)}]");
        }

        // check if cubes are being held or in box
        for (int i = 0; i < _heldCubes.Length; i++)
        {
            if (_heldCubes[i])
            {
                // skip held cubes
                continue;
            }

            // check if cube is in box
            bool inBox = false;
            string cubeIndex = "";
            switch (i)
            {
                case 0:
                    inBox = IsInBox(redCube.transform.position);
                    cubeIndex = "r";
                    break;
                case 1:
                    inBox = IsInBox(yellowCube.transform.position);
                    cubeIndex = "y";
                    break;
                case 2:
                    inBox = IsInBox(blueCube.transform.position);
                    cubeIndex = "b";
                    break;
                case 3:
                    inBox = IsInBox(greenCube.transform.position);
                    cubeIndex = "g";
                    break;
                default:
                    break;
            }

            if (inBox && !_sentCubes[i])
            {
                // send cube index to laptop
                SendCubeIndex(cubeIndex);
                _sentCubes[i] = true;
            }
        }
    }

    private void OnDestroy()
    {
        // close client and server
        _stream.Close();
        _client.Close();
    }

    private bool IsInBox(Vector3 position)
    {
        return position.x > 0.02f && position.x < 0.18f && position.y > -1.2f && position.y < -1f && position.z > 1.3f && position.z < 1.4f;
    }

    private void SendCubeIndex(string cubeIndex)
    {
        byte[] data = Encoding.ASCII.GetBytes(cubeIndex);
        _stream.Write(data, 0, data.Length);
        Debug.Log($"Sent cube index: {cubeIndex}");
    }
}
        if (inBox && !_sentCubes[i])
        {
            // send cube index to laptop
            SendCubeIndex(cubeIndex);
            _sentCubes[i] = true;
        }
    }
private void SendCubeIndex(string cubeIndex)
{
    byte[] data = Encoding.ASCII.GetBytes(cubeIndex);
    _stream.Write(data, 0, data.Length);
    Debug.Log($"Sent cube index: {cubeIndex}");
}
}
