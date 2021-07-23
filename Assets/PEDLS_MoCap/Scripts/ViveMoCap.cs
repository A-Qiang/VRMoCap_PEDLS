using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
using Mocap.DataModel;
using Valve.VR;
using Mocap.IK;
using UnityEngine.UI;
public class ViveMoCap : MonoBehaviour
{
    public Transform bodyMarker;
    public Transform sensor;
    
    Motioner motioner;
    float modelHeight=1.75f;
    float actualHeight;
    bool isCalibrateTpose = false;//states
    

    private void Start()
    {
        Animator animator = transform.GetComponent<Animator>();
        if (animator != null)
        {
            Transform head = animator.GetBoneTransform(HumanBodyBones.Head);
            modelHeight = head.position.y;
        }

        motioner = new Motioner(transform, sensor, bodyMarker);
        
    }
    
    private void Update()
    {
        if (!isCalibrateTpose)
        {
            AdjustScale();
            if (SteamVR_Actions.default_GrabGrip.GetStateDown(SteamVR_Input_Sources.Any) || Input.GetKeyDown(KeyCode.Space))
            {
                motioner.Calibrate();
                HideTrackers();
                isCalibrateTpose = true;
                transform.GetComponent<IKSolverManager>().enabled = true;
            }
        }
        else
        {
            motioner.Update();
        }
    }
    
    void HideTrackers()
    {
        for (int i = 0; i < 6; ++i)
        {
            motioner.sensors[i].GetComponent<MeshRenderer>().enabled = false;
        }
    }

    void AdjustScale()
    {
        actualHeight= motioner.sensors[0].position.y ;
        transform.localScale = Vector3.one*actualHeight / modelHeight;
    }

}
