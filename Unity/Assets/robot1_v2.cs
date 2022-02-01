using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class robot1_v2 : MonoBehaviour
{
    public GameObject camera;
    public GameObject robot_1;
    public GameObject robot_3;
    public GameObject robot_3_bar_left;
    public GameObject robot_3_bar_right;
    public Material robot_3_bar_left_Mat;
    public Material robot_3_bar_right_Mat;
    public float now_time;
    public LineRenderer line;
    // Start is called before the first frame update
    void Start()
    {
        robot_1 = GameObject.Find("robot");
        robot_1.transform.position = new Vector3(0.0f, 0.0f, 0.0f);
        robot_3 = GameObject.Find("robot3");
        robot_3.transform.position = new Vector3(-3.0f, 0.0f, 0.0f);
        robot_3_bar_right = GameObject.Find("bar_right");
        robot_3_bar_left = GameObject.Find("bar_left");
        robot_3_bar_right_Mat = robot_3_bar_right.GetComponent<MeshRenderer>().material;
        robot_3_bar_left_Mat = robot_3_bar_left.GetComponent<MeshRenderer>().material;
        robot_3_bar_right_Mat.color = new Color(1.0f, 0.0f, 0.0f);
        robot_3_bar_left_Mat.color = new Color(1.0f, 0.0f, 0.0f);
        line = robot_3.GetComponent<LineRenderer>();
        line.startWidth = 0.1f;
        line.endWidth = 0.1f;
        line.material = new Material(Shader.Find("Sprites/Default"));
        line.startColor = Color.red;
        line.endColor = Color.red;
        camera = GameObject.Find("Main Camera");
        camera.transform.position = (robot_1.transform.position + robot_3.transform.position) / 2.0f;
        camera.transform.position += new Vector3(0.0f, 2.0f, 0.0f);
    }

    private static float start_time = 3.0f;
    private static float r1_straight_time = 0.0f;
    private static float r1_stop_time = 6.8f + r1_straight_time;
    private static float r3_straight_time = 0.35f;
    private static float r3_stop_time = 6.5f + r3_straight_time;
    private static float r3_right_white_time = 5.7f + r3_straight_time;
    private static float r3_left_white_time = 1.9f + r3_straight_time;
    int count = 0;

    // Update is called once per frame
    void Update()
    {
        camera.transform.position = (robot_1.transform.position + robot_3.transform.position) / 2.0f;
        camera.transform.position += new Vector3(0.0f, 2.0f, 0.0f);
        count += 1;
        line.positionCount = count;
        line.SetPosition(count - 1, robot_3.transform.position);
        now_time = Time.time - start_time;
        if (now_time < -1.5)
        {
            robot_3_bar_right_Mat.color = new Color(1.0f, 0.0f, 0.0f, 0.0f);
            robot_3_bar_left_Mat.color = new Color(1.0f, 0.0f, 0.0f, 0.0f);
            return;
        }
        else if (now_time < 0.0)
        {
            robot_3_bar_right_Mat.color = new Color(1.0f, 0.0f, 0.0f, 1.0f);
            robot_3_bar_left_Mat.color = new Color(1.0f, 0.0f, 0.0f, 1.0f);
            return;
        }

        if (now_time <= r1_straight_time)
        {
            robot_1.transform.position = new Vector3(now_time * 1.5f, 0.0f, 0.0f);
        }
        else if (now_time <= r1_stop_time)
        {
            float x_1 = r1_straight_time * 1.5f + 5.0f * Mathf.Sin(2.0f * Mathf.PI * (now_time - r1_straight_time) * 0.05f);
            float z_1 = 5.0f * (1.0f - Mathf.Cos(2.0f * Mathf.PI * (now_time - r1_straight_time) * 0.05f));
            robot_1.transform.position = new Vector3(x_1, 0.0f, z_1);
            robot_1.transform.localRotation = Quaternion.Euler(0.0f, -(now_time - r1_straight_time) * 18.0f, 0.0f);
        }

        if (now_time <= r3_straight_time)
        {
            robot_3.transform.position = new Vector3(-3.0f + now_time * 1.5f, 0.0f, 0.0f);
        }
        else if (now_time <= r3_stop_time)
        {
           
            float x_3 = (now_time - r3_straight_time) * 0.15f - 3.0f + r3_straight_time * 1.5f + 5.0f * Mathf.Sin(2.0f * Mathf.PI * (now_time - r3_straight_time) * 0.04f);
            float z_3 = 5.0f * (1.0f - Mathf.Cos(2.0f * Mathf.PI * (now_time - r3_straight_time) * 0.04f));
            robot_3.transform.position = new Vector3(x_3, 0.0f, z_3);
            robot_3.transform.localRotation = Quaternion.Euler(0.0f, -(now_time - r3_straight_time) * 12.8f, 0.0f);
            
        }

        /*if (now_time >= r3_right_white_time)
        {
            robot_3_bar_right_Mat.color = new Color(1.0f, 1.0f, 1.0f);
        }*/
        if (now_time >= r3_left_white_time)
        {
            robot_3_bar_left_Mat.color = new Color(1.0f, 1.0f, 1.0f);
        }
    }
}
