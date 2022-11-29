using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetDisplay : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        hide();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void show()
    {
        gameObject.GetComponent<Renderer>().enabled = true;
    }

    void hide()
    {
        gameObject.GetComponent<Renderer>().enabled = false;
    }
}
