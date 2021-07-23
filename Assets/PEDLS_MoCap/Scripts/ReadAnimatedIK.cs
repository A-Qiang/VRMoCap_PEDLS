using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mocap.DataProcess;
using Mocap.IK;
public class ReadAnimatedIK : MonoBehaviour
{
    public Transform targetRoot;
    public Transform modelRoot;
    public Transform trackerRoot;
    HumanSkeleton tracker;
    HumanSkeleton target;
    HumanSkeleton initModel;

    Vector3[] offsetPos;
    Quaternion[] offsetRot;
    
    // Start is called before the first frame update
    void Start()
    {
        tracker = new HumanSkeleton();
        target = new HumanSkeleton();
        initModel = new HumanSkeleton();
        target.FindTransforms(targetRoot);
        tracker.GetTransforms(trackerRoot);
        initModel.GetTransforms(modelRoot);

        offsetPos = new Vector3[tracker.Transforms.Length];
        offsetRot = new Quaternion[tracker.Transforms.Length];

        for(int i = 0; i < offsetPos.Length; ++i)
        {
            if (target.Transforms[i] != null)
            {
                offsetPos[i] = initModel.Transforms[i].position - tracker.Transforms[i].position;
                target.Transforms[i].position = target.Transforms[i].position;
                target.Transforms[i].rotation = initModel.Transforms[i].rotation;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < tracker.Transforms.Length; ++i)
        {
            if (target.Transforms[i] != null)
            {
                target.Transforms[i].rotation = tracker.Transforms[i].rotation;
                target.Transforms[i].position = tracker.Transforms[i].position + offsetPos[i];
            }
        }
    }
}
