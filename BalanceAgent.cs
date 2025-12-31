using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// <summary>
/// BalanceAgent - Matched to Kevin Iglesias model hierarchy with CharacterJoint
/// 
/// YOUR HIERARCHY:
/// Fighter
/// └── Rig
///     └── B-root
///         └── B-hips (Rigidbody - ROOT)
///             ├── B-spine (Rigidbody + CharacterJoint)
///             │   └── B-chest (EMPTY)
///             │       └── B-head (Rigidbody + CharacterJoint)
///             │       └── B-upperArm.L/R → B-forearm.L/R
///             ├── B-thigh.L (Rigidbody + CharacterJoint)
///             │   └── B-shin.L (Rigidbody + CharacterJoint)
///             │       └── B-foot.L (Transform only - for ground detection)
///             └── B-thigh.R → B-shin.R → B-foot.R
/// 
/// Observations: 45
/// Actions: 10 (torques applied to rigidbodies)
/// </summary>
public class BalanceAgent : Agent
{
    [Header("═══════ CORE BODY (REQUIRED) ═══════")]
    [Tooltip("B-hips - ROOT rigidbody, no CharacterJoint")]
    public Rigidbody hips;
    
    [Tooltip("B-spine - has CharacterJoint")]
    public Rigidbody spine;
    
    [Tooltip("B-head - has CharacterJoint")]
    public Rigidbody head;
    
    [Header("═══════ LEGS (REQUIRED) ═══════")]
    [Tooltip("B-thigh.L")]
    public Rigidbody leftThigh;
    
    [Tooltip("B-shin.L")]
    public Rigidbody leftShin;
    
    [Tooltip("B-thigh.R")]
    public Rigidbody rightThigh;
    
    [Tooltip("B-shin.R")]
    public Rigidbody rightShin;
    
    [Header("═══════ FEET (Transform only - for ground check) ═══════")]
    [Tooltip("B-foot.L transform")]
    public Transform leftFoot;
    
    [Tooltip("B-foot.R transform")]
    public Transform rightFoot;
    
    [Header("═══════ ARMS (Optional - for observation only) ═══════")]
    [Tooltip("B-upperArm.L")]
    public Rigidbody leftUpperArm;
    
    [Tooltip("B-upperArm.R")]
    public Rigidbody rightUpperArm;
    
    [Header("═══════ GROUND DETECTION ═══════")]
    public LayerMask groundLayer;
    public float footGroundCheckDistance = 0.15f;
    
    [Header("═══════ TORQUE SETTINGS ═══════")]
    [Tooltip("Maximum torque applied to joints")]
    public float maxTorque = 200f;
    
    [Header("═══════ EPISODE SETTINGS ═══════")]
    public int minStepsBeforeTermination = 200;
    public int maxEpisodeSteps = 2000;
    
    [Header("═══════ TARGET VALUES ═══════")]
    public float targetHipHeight = 0.95f;
    public float minHipHeight = 0.3f;
    
    // Runtime
    private List<Rigidbody> allBodies;
    private BodyState[] initialStates;
    private Vector3 initialHipsWorldPos;
    private Quaternion initialHipsWorldRot;
    
    private bool leftFootGrounded;
    private bool rightFootGrounded;
    private Vector3 centerOfMass;
    private int stepCount;
    
    private struct BodyState
    {
        public Vector3 localPos;
        public Quaternion localRot;
    }

    public override void Initialize()
    {
        // Gather all rigidbodies
        allBodies = new List<Rigidbody>();
        
        // Required
        if (hips) allBodies.Add(hips);
        if (spine) allBodies.Add(spine);
        if (head) allBodies.Add(head);
        if (leftThigh) allBodies.Add(leftThigh);
        if (leftShin) allBodies.Add(leftShin);
        if (rightThigh) allBodies.Add(rightThigh);
        if (rightShin) allBodies.Add(rightShin);
        
        // Optional arms
        if (leftUpperArm) allBodies.Add(leftUpperArm);
        if (rightUpperArm) allBodies.Add(rightUpperArm);
        
        float totalMass = 0f;
        foreach (var rb in allBodies)
            totalMass += rb.mass;
        
        Debug.Log($"[BalanceAgent] Initialized: {allBodies.Count} rigidbodies, {totalMass:F1} kg total mass");
        
        StoreInitialState();
    }
    
    private void StoreInitialState()
    {
        // Get ALL rigidbodies in hierarchy for proper reset
        Rigidbody[] allRbs = GetComponentsInChildren<Rigidbody>();
        initialStates = new BodyState[allRbs.Length];
        
        for (int i = 0; i < allRbs.Length; i++)
        {
            initialStates[i] = new BodyState
            {
                localPos = allRbs[i].transform.localPosition,
                localRot = allRbs[i].transform.localRotation
            };
        }
        
        if (hips)
        {
            initialHipsWorldPos = hips.transform.position;
            initialHipsWorldRot = hips.transform.rotation;
        }
    }
    
    public override void OnEpisodeBegin()
    {
        stepCount = 0;
        
        // Reset all rigidbodies
        Rigidbody[] allRbs = GetComponentsInChildren<Rigidbody>();
        
        for (int i = 0; i < allRbs.Length && i < initialStates.Length; i++)
        {
            allRbs[i].transform.localPosition = initialStates[i].localPos;
            allRbs[i].transform.localRotation = initialStates[i].localRot;
            allRbs[i].velocity = Vector3.zero;
            allRbs[i].angularVelocity = Vector3.zero;
        }
        
        // Reset root position
        if (hips)
        {
            hips.transform.position = initialHipsWorldPos;
            hips.transform.rotation = initialHipsWorldRot;
        }
    }

    /// <summary>
    /// 45 Observations total
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        UpdateGroundContact();
        centerOfMass = CalculateCenterOfMass();
        
        // ═══════ HIPS (10) ═══════
        sensor.AddObservation(hips.position.y / targetHipHeight);           // 1
        sensor.AddObservation(hips.transform.up);                            // 3
        sensor.AddObservation(hips.velocity / 5f);                           // 3
        sensor.AddObservation(hips.angularVelocity / 10f);                   // 3
        
        // ═══════ SPINE (7) ═══════
        float spineUp = spine ? Vector3.Dot(spine.transform.up, Vector3.up) : 1f;
        sensor.AddObservation(spineUp);                                      // 1
        sensor.AddObservation(spine ? spine.transform.forward : Vector3.forward); // 3
        sensor.AddObservation(spine ? spine.angularVelocity / 10f : Vector3.zero); // 3
        
        // ═══════ HEAD (4) ═══════
        float headHeight = head ? head.position.y : hips.position.y + 0.5f;
        float headUp = head ? Vector3.Dot(head.transform.up, Vector3.up) : 1f;
        sensor.AddObservation(headHeight / 1.8f);                            // 1
        sensor.AddObservation(headUp);                                       // 1
        sensor.AddObservation(head ? head.velocity.y / 5f : 0f);             // 1
        sensor.AddObservation(head ? head.angularVelocity.magnitude / 10f : 0f); // 1
        
        // ═══════ LEFT LEG (6) ═══════
        ObserveLimb(sensor, leftThigh);  // 3
        ObserveLimb(sensor, leftShin);   // 3
        
        // ═══════ RIGHT LEG (6) ═══════
        ObserveLimb(sensor, rightThigh); // 3
        ObserveLimb(sensor, rightShin);  // 3
        
        // ═══════ GROUND CONTACT (2) ═══════
        sensor.AddObservation(leftFootGrounded ? 1f : 0f);                   // 1
        sensor.AddObservation(rightFootGrounded ? 1f : 0f);                  // 1
        
        // ═══════ CENTER OF MASS (6) ═══════
        Vector3 comRel = centerOfMass - hips.position;
        sensor.AddObservation(comRel.x / 0.3f);                              // 1
        sensor.AddObservation(comRel.y / 0.3f);                              // 1
        sensor.AddObservation(comRel.z / 0.3f);                              // 1
        sensor.AddObservation(centerOfMass.y / targetHipHeight);             // 1
        sensor.AddObservation(Mathf.Clamp01(hips.position.y / targetHipHeight)); // 1
        sensor.AddObservation(hips.velocity.y / 5f);                         // 1
        
        // ═══════ BALANCE (4) ═══════
        if (leftFoot && rightFoot)
        {
            Vector3 support = (leftFoot.position + rightFoot.position) * 0.5f;
            Vector3 comGround = new Vector3(centerOfMass.x, 0, centerOfMass.z);
            Vector3 supGround = new Vector3(support.x, 0, support.z);
            float dist = Vector3.Distance(comGround, supGround);
            Vector3 toSupport = dist > 0.01f ? (supGround - comGround).normalized : Vector3.zero;
            
            sensor.AddObservation(Mathf.Clamp01(1f - dist / 0.3f));          // 1
            sensor.AddObservation(toSupport.x);                               // 1
            sensor.AddObservation(toSupport.z);                               // 1
            sensor.AddObservation(dist / 0.3f);                               // 1
        }
        else
        {
            sensor.AddObservation(0.5f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
        
        // TOTAL: 10 + 7 + 4 + 6 + 6 + 2 + 6 + 4 = 45
    }
    
    private void ObserveLimb(VectorSensor sensor, Rigidbody limb)
    {
        if (limb && hips)
        {
            Quaternion localRot = Quaternion.Inverse(hips.rotation) * limb.rotation;
            sensor.AddObservation(localRot.x);                               // 1
            sensor.AddObservation(localRot.w);                               // 1
            sensor.AddObservation(limb.angularVelocity.magnitude / 10f);     // 1
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
        }
    }

    /// <summary>
    /// 10 Actions - Torques applied to rigidbodies
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        stepCount++;
        var act = actions.ContinuousActions;
        int i = 0;
        
        // ═══════ SPINE: pitch, roll (2) ═══════
        if (spine)
        {
            spine.AddRelativeTorque(new Vector3(
                act[i++] * maxTorque,      // pitch (forward/back)
                0f,
                act[i++] * maxTorque       // roll (side to side)
            ));
        }
        else { i += 2; }
        
        // ═══════ LEFT THIGH: pitch, roll (2) ═══════
        if (leftThigh)
        {
            leftThigh.AddRelativeTorque(new Vector3(
                act[i++] * maxTorque,
                0f,
                act[i++] * maxTorque * 0.5f
            ));
        }
        else { i += 2; }
        
        // ═══════ LEFT SHIN: pitch only (1) ═══════
        if (leftShin)
        {
            leftShin.AddRelativeTorque(new Vector3(act[i++] * maxTorque, 0f, 0f));
        }
        else { i += 1; }
        
        // ═══════ RIGHT THIGH: pitch, roll (2) ═══════
        if (rightThigh)
        {
            rightThigh.AddRelativeTorque(new Vector3(
                act[i++] * maxTorque,
                0f,
                act[i++] * maxTorque * 0.5f
            ));
        }
        else { i += 2; }
        
        // ═══════ RIGHT SHIN: pitch only (1) ═══════
        if (rightShin)
        {
            rightShin.AddRelativeTorque(new Vector3(act[i++] * maxTorque, 0f, 0f));
        }
        else { i += 1; }
        
        // ═══════ HEAD: pitch, roll (2) ═══════
        if (head)
        {
            head.AddRelativeTorque(new Vector3(
                act[i++] * maxTorque * 0.3f,
                0f,
                act[i++] * maxTorque * 0.3f
            ));
        }
        
        // TOTAL: 2 + 2 + 1 + 2 + 1 + 2 = 10 actions
        
        CalculateRewards();
    }

    private void CalculateRewards()
    {
        float reward = 0f;
        
        float hipHeight = hips.position.y;
        float headHeight = head ? head.position.y : hipHeight + 0.5f;
        float spineUp = spine ? Vector3.Dot(spine.transform.up, Vector3.up) : 0f;
        float hipsUp = Vector3.Dot(hips.transform.up, Vector3.up);
        
        // ═══════════════════════════════════════════════════════════════════
        // PRIMARY: HEIGHT REWARD (prevents lying down!)
        // ═══════════════════════════════════════════════════════════════════
        float heightRatio = Mathf.Clamp01(hipHeight / targetHipHeight);
        reward += heightRatio * 0.5f;
        
        // Big bonus for being tall (above 70% target height)
        if (heightRatio > 0.7f)
        {
            reward += 0.3f;
        }
        
        // ═══════════════════════════════════════════════════════════════════
        // SECONDARY: UPRIGHT TORSO
        // ═══════════════════════════════════════════════════════════════════
        float uprightBonus = (Mathf.Clamp01(spineUp) + Mathf.Clamp01(hipsUp)) * 0.5f;
        reward += uprightBonus * 0.15f;
        
        // ═══════════════════════════════════════════════════════════════════
        // TERTIARY: FEET ON GROUND
        // ═══════════════════════════════════════════════════════════════════
        if (leftFootGrounded && rightFootGrounded)
            reward += 0.05f;
        else if (leftFootGrounded || rightFootGrounded)
            reward += 0.02f;
        
        // ═══════════════════════════════════════════════════════════════════
        // PENALTIES
        // ═══════════════════════════════════════════════════════════════════
        
        // STRONG penalty for being low - this is key to prevent lying down!
        if (hipHeight < 0.5f)
        {
            reward -= 0.4f;
        }
        else if (hipHeight < 0.7f)
        {
            reward -= 0.2f;
        }
        
        // Penalty for tilted torso
        if (spineUp < 0.5f)
        {
            reward -= 0.15f;
        }
        
        // Small penalty for excessive movement
        if (hips.velocity.magnitude > 2f)
        {
            reward -= 0.05f;
        }
        
        // ═══════════════════════════════════════════════════════════════════
        // EPISODE TERMINATION
        // ═══════════════════════════════════════════════════════════════════
        
        bool collapsed = false;
        
        if (hipHeight < minHipHeight)
            collapsed = true;
        
        if (headHeight < 0.25f)
            collapsed = true;
        
        if (spineUp < 0.1f)
            collapsed = true;
        
        // Only terminate after minimum exploration time
        if (collapsed && stepCount >= minStepsBeforeTermination)
        {
            reward -= 1.0f;
            AddReward(reward);
            EndEpisode();
            return;
        }
        
        // Success: survived full episode!
        if (stepCount >= maxEpisodeSteps)
        {
            reward += 2.0f;
            AddReward(reward);
            EndEpisode();
            return;
        }
        
        AddReward(reward);
    }

    private void UpdateGroundContact()
    {
        float dist = footGroundCheckDistance + 0.1f;
        Vector3 up = Vector3.up * 0.1f;
        
        leftFootGrounded = leftFoot && Physics.Raycast(leftFoot.position + up, Vector3.down, dist, groundLayer);
        rightFootGrounded = rightFoot && Physics.Raycast(rightFoot.position + up, Vector3.down, dist, groundLayer);
    }
    
    private Vector3 CalculateCenterOfMass()
    {
        Vector3 com = Vector3.zero;
        float mass = 0f;
        
        foreach (var rb in allBodies)
        {
            if (rb)
            {
                com += rb.worldCenterOfMass * rb.mass;
                mass += rb.mass;
            }
        }
        
        return mass > 0f ? com / mass : hips.position;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        for (int i = 0; i < act.Length; i++)
            act[i] = 0f;
        
        // Arrow keys for manual testing
        if (Input.GetKey(KeyCode.UpArrow)) act[0] = -0.5f;    // spine forward
        if (Input.GetKey(KeyCode.DownArrow)) act[0] = 0.5f;   // spine back
        if (Input.GetKey(KeyCode.LeftArrow)) act[1] = -0.5f;  // spine left
        if (Input.GetKey(KeyCode.RightArrow)) act[1] = 0.5f;  // spine right
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || !hips) return;
        
        // Center of mass
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(centerOfMass, 0.08f);
        
        // Height indicator
        float ratio = hips.position.y / targetHipHeight;
        Gizmos.color = ratio > 0.7f ? Color.green : Color.red;
        Gizmos.DrawLine(hips.position, hips.position + Vector3.up * 0.2f);
        
        // Foot contact
        if (leftFoot)
        {
            Gizmos.color = leftFootGrounded ? Color.green : Color.red;
            Gizmos.DrawWireSphere(leftFoot.position, 0.03f);
        }
        if (rightFoot)
        {
            Gizmos.color = rightFootGrounded ? Color.green : Color.red;
            Gizmos.DrawWireSphere(rightFoot.position, 0.03f);
        }
    }
}
