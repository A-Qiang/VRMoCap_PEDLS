using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TextFixed : MonoBehaviour
{
    public Transform obj;
    Vector3 off;
    // Start is called before the first frame update
    void Start()
    {
       off = transform.position-obj.position;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = obj.position + off;
    }
}
