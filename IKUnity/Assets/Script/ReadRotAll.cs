using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class ReadRotAll : MonoBehaviour
{
    public GameObject Marker;

    public float[,] bvh_data = new float[9636, 84];//读取BVH数据
    public string[] datastr = new string[84];

    public float[,] raw_marker = new float[9636, 123];//读取BVH数据
    public string[] datastr_m = new string[123];

    public float[,] pre_hip = new float[9636, 3];//读取BVH数据
    public string[] datastr_hip = new string[3];

    public int frame_index = 0;
    public GameObject[] Joint = new GameObject[21];
    public GameObject Hip;//根节点
    // Start is called before the first frame update
    void Start()
    {

        //读取BVH数据
        FileStream fs = new FileStream("Assets/q_left_all.txt", FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs, System.Text.Encoding.Default);
        string str;
        for (int i = 0; (str = sr.ReadLine()) != null; i++)
        {
            datastr = str.Split(' ');
            //Debug.Log(datastr);
            for (int j = 0; j < 84; j++)
            {
                bvh_data[i, j] = float.Parse(datastr[j]);
            }
        }
        sr.Close();

        ////读取marker数据
        //fs = new FileStream("Assets/raw_data_new.txt", FileMode.Open, FileAccess.Read);
        //sr = new StreamReader(fs, System.Text.Encoding.Default);
        //for (int i = 0; (str = sr.ReadLine()) != null; i++)
        //{
        //    datastr_m = str.Split(' ');
        //    //Debug.Log(datastr);
        //    for (int j = 0; j < 123; j++)
        //    {
        //        raw_marker[i, j] = float.Parse(datastr_m[j]);
        //    }
        //}
        //sr.Close();

        //根节点位置
        fs = new FileStream("Assets/hip_p.txt", FileMode.Open, FileAccess.Read);
        sr = new StreamReader(fs, System.Text.Encoding.Default);
        for (int i = 0; (str = sr.ReadLine()) != null; i++)
        {
            datastr_hip = str.Split(' ');
            //Debug.Log(datastr);
            for (int j = 0; j < 3; j++)
            {
                pre_hip[i, j] = float.Parse(datastr_hip[j]);
            }
        }
        sr.Close();

        
    }

    // Update is called once per frame
    void Update()
    {
        if (frame_index >= 9636) frame_index = 0;
        Hip.GetComponent<Transform>().position = new Vector3(-pre_hip[frame_index, 0], pre_hip[frame_index, 1], pre_hip[frame_index, 2]);
        for (int i=0;i<21;i++)
        {
            Quaternion q;
            q.w = bvh_data[frame_index, i * 4]; q.x = bvh_data[frame_index, i * 4 + 1];
            q.y =bvh_data[frame_index, i * 4 + 2]; q.z = bvh_data[frame_index, i * 4 + 3];
            Joint[i].GetComponent<Transform>().rotation = Quaternion.Inverse(q);
        }


        ////生成Marker
        //for (int i = 0; i < 41; i++)
        //{
        //    Instantiate(Marker, new Vector3(-raw_marker[frame_index, i * 3], raw_marker[frame_index, i * 3 + 1], raw_marker[frame_index, i * 3+2]), Quaternion.identity);

        //}

        frame_index++;
        
    }
}
