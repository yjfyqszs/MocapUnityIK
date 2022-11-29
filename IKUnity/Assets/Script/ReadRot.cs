using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class ReadRot : MonoBehaviour
{
    public string[] datastr = new string[84];
    public GameObject[] Joint = new GameObject[21];
    // Start is called before the first frame update
    void Start()
    {
        FileStream fs = new FileStream("Assets/q_left.txt", FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs, System.Text.Encoding.Default);
        string str;
        str = sr.ReadLine();
        datastr = str.Split(' ');
        sr.Close();
        
    }

    // Update is called once per frame
    void Update()
    {
        for (int i=0;i<21;i++)
        {
            Quaternion q;
            q.w = float.Parse(datastr[i * 4]); q.x = float.Parse(datastr[i * 4 + 1]);
            q.y= float.Parse(datastr[i * 4 + 2]); q.z = float.Parse(datastr[i * 4 + 3]);
            Joint[i].GetComponent<Transform>().rotation = Quaternion.Inverse(q);
        }
        
    }
}
