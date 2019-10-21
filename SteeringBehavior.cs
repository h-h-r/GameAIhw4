using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is the place to put all of the various steering behavior methods we're going
/// to be using. Probably best to put them all here, not in NPCController.
/// </summary>

public class SteeringBehavior : MonoBehaviour {

    // The agent at hand here, and whatever target it is dealing with
    public NPCController agent;
    public NPCController target;
    public NPCController red;

    // Below are a bunch of variable declarations that will be used for the next few
    // assignments. Only a few of them are needed for the first assignment.

    // For pursue and evade functions
    public float maxPrediction;
    public float maxAcceleration;

    // For arrive function
    public float maxSpeed;
    public float targetRadiusL;
    public float slowRadiusL;
    public float timeToTarget;

    // For Face function
    public float maxRotation;
    public float maxAngularAcceleration;
    public float targetRadiusA;
    public float slowRadiusA;

    // For wander function
    public float wanderOffset;
    public float wanderRadius;
    public float wanderRate;
    private float wanderOrientation;

    // Holds the path to follow
    public GameObject[] Path;
    public int current = 0;

    protected void Start() {
        agent = GetComponent<NPCController>();
        wanderOrientation = agent.orientation;
        red = GameObject.FindGameObjectWithTag("Red").GetComponent<NPCController>();
    }

    public Vector3 Seek() {
        return new Vector3(0f, 0f, 0f);
    }

    public Vector3 Flee()
    {
        return new Vector3(0f, 0f, 0f);
    }


    // Calculate the target to pursue
    public Vector3 Pursue() {
        return new Vector3(0f, 0f, 0f);
    }

    public float Face()
    {
        return 0f;
    }

    public Vector3 SeparationAcc(string tag)
    {

        GameObject[] group = GameObject.FindGameObjectsWithTag(tag);
        //print(group.Length+">>>\n");

        Vector3 linear_acc = new Vector3(0f, 0f, 0f);

        for(int i = 0; i < group.Length; i++)
        {
            Vector3 direction = agent.position - group[i].GetComponent<NPCController>().position;
            //if agent with group[i] too close
            if (direction.magnitude>0.00001 && direction.magnitude < 2f)
            {
                linear_acc += direction / direction.magnitude /direction.magnitude;
                //print("i:" + i + ", linearacc " + linear_acc + "\n");
            }
        }
        //check for red
        if ((agent.position - red.position).magnitude < 2f)
        {
            linear_acc += 5*(agent.position - red.position).normalized;
        }
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print("linear acc = " + linear_acc + "\n");
        return linear_acc;
    }


    public Vector3 CohesionAcc()
    {
       
        //return Arrive(red.position - agent.position);

        //Vector3 linear_acc = new Vector3(0f, 0f, 0f);

        //return linear_acc;
        GameObject[] group = GameObject.FindGameObjectsWithTag("Wolf");
        Vector3 avgPosition = new Vector3(0f, 0f, 0f);
        for (int i = 0; i < group.Length; i++)
        {
            avgPosition += group[i].GetComponent<NPCController>().position;
        }
        avgPosition += group.Length * red.position;
        avgPosition /= (group.Length*2);
        return Arrive(avgPosition - agent.position);

    }

    public Vector3 Align(string tag)
    {

        GameObject[] group = GameObject.FindGameObjectsWithTag(tag);
        //print(group.Length+">>>\n");

        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        Vector3 avg_velocity = new Vector3(0f, 0f, 0f);
        int count = 0;

        for (int i = 0; i < group.Length; i++)
        {
            Vector3 direction = agent.position - group[i].GetComponent<NPCController>().position;
            //if agent with group[i] too close
            if (direction.magnitude > 0.00001 && direction.magnitude < 4f)
            {
                avg_velocity += group[i].GetComponent<NPCController>().velocity;
                count += 1;
                //linear_acc += direction / direction.magnitude / direction.magnitude;
                //print("i:" + i + ", linearacc " + linear_acc + "\n");
            }
        }

        if ((red.position - agent.position).magnitude < 4f)
        {
            avg_velocity += red.velocity;
            count+=1;
        }

        if (count == 0)
        {
            return linear_acc;
        }
        else
        {
            avg_velocity /= count;
            linear_acc = (avg_velocity - agent.velocity).normalized * maxAcceleration;
            return linear_acc;
        }
        //check for red
        if ((agent.position - red.position).magnitude < 2f)
        {
            linear_acc += 5 * (agent.position - red.position).normalized;
        }
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        //print("linear acc = " + linear_acc + "\n");
        return linear_acc;
    }



    public Vector3 FlockingAcc()
    {
        Vector3 linear_acc = new Vector3(0f, 0f, 0f);
        linear_acc += 3* SeparationAcc("Wolf");
        linear_acc += 1* CohesionAcc();
        linear_acc += 2 * Align("Wolf");
        linear_acc = linear_acc.normalized;
        linear_acc *= maxAcceleration;
        return linear_acc;
    }


    // ETC.

    //Haoran
    //dynamic arrive given target
    //return linear acc
    public Vector3 Arrive()
    {
        Vector3 direction = target.position - agent.position;
        return Arrive(direction);
    }

    //Haoran
    //dynamic arrive given direction
    //return linear acc
    public Vector3 Arrive(Vector3 direction)
    {
        // Vector3 direction = target.position - agent.position;

        if (direction.magnitude <= slowRadiusL)
        {
            //agent.label.text = "dynamic arrive\n<In slowRadiusL>";
            //agent.DestroyPoints();
            //agent.DrawCircle(target.position, slowRadiusL);
        }
        else
        {
            agent.DestroyPoints();
        }

        // stop if arrive (in target radius) and return zero linear_acc
        if (direction.magnitude < targetRadiusA )
        {
            //agent.label.text = "dynamic arrive\n<In targetRadiusA>";
            agent.velocity = Vector3.zero;
            return Vector3.zero;
        }

        //calculate appropriate speed
        float speed = (direction.magnitude > slowRadiusL ? maxSpeed : maxSpeed * direction.magnitude / slowRadiusL);

        //apply direction
        direction.Normalize();
        Vector3 velocity = direction * speed;

        // calculate linear_acc
        Vector3 linear_acc = (velocity - agent.velocity) / timeToTarget;

        // clip linear_acc
        if (linear_acc.magnitude > maxAcceleration)
        {
            linear_acc.Normalize();
            linear_acc *= maxAcceleration;
        }

        return linear_acc;
    }

    //Haoran
    //face to given direction
    //return angular acc
    public float FaceTo(Vector3 direction)
    {
        // Check for a zero direction, and make no change if so
        if (direction.magnitude == 0)
        {
            return 0;
        }

        // Get anount of angle need to rotate
        float rotationAmount = Mathf.Atan2(direction.x, direction.z) - agent.orientation;
        //agent.orientaion range [-inf,inf]

        // clip to (-pi, pi) interval
        while (rotationAmount > Mathf.PI)
        {
            rotationAmount -= 2 * Mathf.PI;
        }
        while (rotationAmount < -Mathf.PI)
        {
            rotationAmount += 2 * Mathf.PI;
        }

        // if already facing target, set angular speed to zero
        if (Mathf.Abs(rotationAmount) < targetRadiusA)
        {
            agent.rotation = 0;
        }

        // greater than slowRadius => clip to max rotation speed
        // less than slowRadius => clip to scaled rotation speed 
        float rotationSpeed = (rotationAmount > slowRadiusA ? maxRotation : maxRotation * Mathf.Abs(rotationAmount) / slowRadiusA);

        // get the correct rotation direction
        rotationSpeed *= rotationAmount / Mathf.Abs(rotationAmount);

        // calculate the rotation acceleration
        float angular_acc = rotationSpeed - agent.rotation;
        angular_acc /= timeToTarget;

        // clip to max angular acc if needed
        if (Mathf.Abs(angular_acc) > maxAngularAcceleration)
        {
            angular_acc /= Mathf.Abs(angular_acc);
            angular_acc *= maxAngularAcceleration;
        }

        return angular_acc;
    }


}
