using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class AgentBehaviour : Agent
{
    Rigidbody rBody;
    public Transform Target;
    public Transform Button;
    public RayPerceptionSensorComponent3D perceptionSensor;
    public float speedMultiplier = 10;

    public bool button_pressed;
    public Vector3 prev_position;

    public float mapWidth = 28f; // Define map dimensions
    public float mapHeight = 28f;
    public LayerMask obstacleLayer; // Layer of the obstacles

    public Quaternion lastRotation;
    // Start is called before the first frame update
    void Start()
    {
        rBody = this.GetComponent<Rigidbody>();
        rBody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
    }

    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (this.transform.localPosition.y < -1)
        {
            this.transform.rotation = new Quaternion(0f, 0f, 0f, 0f);
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0, 0);
        }

        prev_position = transform.position;
        
        // Hide target
        Target.localPosition = new Vector3(0, -2, 0);
        lastRotation = transform.rotation;

        // Position randomly the button
        Button.localPosition = SpawnObject();
        Button.localPosition = new Vector3(Button.localPosition.x, -0.55f, Button.localPosition.z);

        button_pressed = false;
    }

    private void MoveAgent(ActionBuffers actionBuffers)
    {
        float move_forward = actionBuffers.ContinuousActions[1];
        float rotate = actionBuffers.ContinuousActions[0];

        rBody.MovePosition(transform.position + transform.forward*move_forward*speedMultiplier*Time.deltaTime);
        this.transform.Rotate(0f, rotate*speedMultiplier, 0f, Space.Self);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(button_pressed ? 1 : 0);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float distanceToGoal = 0f;
        float reward = 0f;
        string goal = "";
        string detectedTag = "";
        var detectableTag = perceptionSensor.DetectableTags;

        // update agent movement
        MoveAgent(actionBuffers);

        float rotationChange = Quaternion.Angle(transform.rotation, lastRotation);
        lastRotation = transform.rotation;
        reward += rotationChange/25;

        // Get the ray perception data
        RayPerceptionInput rayInput = perceptionSensor.GetRayPerceptionInput();
        RayPerceptionOutput rayOutput = RayPerceptionSensor.Perceive(rayInput);

        if(button_pressed)
        {
            goal = "Target";
            distanceToGoal = Vector3.Distance(this.transform.localPosition, Target.localPosition);
        }
        else
        {
            goal = "Button";
            distanceToGoal = Vector3.Distance(this.transform.localPosition, Button.localPosition);
        }

        // Iterate through each ray's data
        int counter = 0;
        int penalty_counter = 0;
        foreach (var rayOutputResult in rayOutput.RayOutputs)
        {
            counter += 1;
            // Check if the ray hit something
            if (rayOutputResult.HitTaggedObject)
            {
                // Get the tag of the hit object
                detectedTag = detectableTag[rayOutputResult.HitTagIndex];

                // Calculate reward or penalty based on the object tag
                if (detectedTag == goal)
                {
                    reward += (1.0f - rayOutputResult.HitFraction)/(button_pressed ? 10.0f : 30.0f);  // Reward for detecting the goal
                }
                else if (rayOutputResult.HitFraction < 0.05f)
                {
                    penalty_counter += 1;
                }
            }
        }

        if (penalty_counter == counter)
        {
            reward -= 10.0f;
        }        
        reward -= 0.01f;

        SetReward(reward);
        print(reward);

        // Reached target
        if (distanceToGoal < 1.42f)
        {
            
            if (!button_pressed)
            {
                SetReward(200.0f);
                button_pressed = true;
                // show target
                Target.localPosition = SpawnObject();
            }
            else
            {
                SetReward(400.0f);
                EndEpisode();
            }
        }

    }

    private Vector3 SpawnObject()
    {
        Vector3 randomPosition;
        bool isPositionValid;

        do
        {
            // Generate a random position within the map bounds
            float x = 0;
            while(Math.Abs(x) < 4)
            {
                x = UnityEngine.Random.Range(-mapWidth / 2, mapWidth / 2);
            }

            float z = UnityEngine.Random.Range(-mapHeight / 2, mapHeight / 2);
            
            randomPosition = new Vector3(x, 0, z); // Set y to 0 (ground level)

            // Check if the position overlaps with any obstacles
            isPositionValid = !Physics.CheckSphere(randomPosition, 2.5f, obstacleLayer);

            if (button_pressed && Vector3.Distance(Button.position, randomPosition) < 10.0f)
            {
                isPositionValid=false;
            }

        } while (!isPositionValid); // Repeat if the position is invalid

        // return valid position
        return randomPosition;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = -Input.GetAxis("Vertical");
    }


}
