# unity-ragdoll-balance-mlagents
Training a physics-based humanoid ragdoll to stand using Unity ML-Agents (PPO)
# Unity Ragdoll Balance Training (ML-Agents PPO)

This project demonstrates how to train a **full humanoid ragdoll** to stand upright
using **Unity ML-Agents (PPO)** and **pure physics** — no animations, no mocap.

The agent learns balance through reward shaping, joint spring control, and
multi-agent parallel training.

---

##  Project Goals
- Train a humanoid ragdoll to stand using reinforcement learning
- Use Unity’s built-in Ragdoll Wizard + CharacterJoints
- Avoid animation-driven or scripted behavior
- Document the entire process clearly and reproducibly

---

##  Full Step-by-Step Guide (PDF)

A complete **28-page guide** covering:
- Installation (Unity, Python, ML-Agents)
- Ragdoll setup and joint configuration
- Reward & penalty design
- Training phases and expectations
- Common failure modes and fixes

 **Read the full guide here:**  
[AI Ragdoll Balance Training Guide (PDF)](./AI_Ragdoll_Balance_Training_Guide.pdf)

---

##  Training Overview

**Algorithm:** PPO (ML-Agents)  
**Observations:** 45 (hip height, torso orientation, velocities, contacts)  
**Actions:** Continuous joint torque modulation  
**Training Style:** Multi-agent parallel training (hundreds of agents)

### Training Phases
1. **Gen 1:** Learn to stay upright at all
2. **Gen 2:** Improve posture and stability using initialized weights

---

## 🛠 Key Techniques Used
- CharacterJoint springs (instead of ConfigurableJoint)
- Height-focused reward shaping
- Early termination for collapse
- Ground contact rewards
- Transfer learning via ONNX initialization

---

##  Repository Structure

##  Known Limitations
- Standing only (no locomotion yet)
- Requires careful joint tuning
- Training time is significant

---

##  Feedback Welcome
This is an evolving project.  
I’d love feedback on:
- Reward shaping
- Observation space
- Joint control strategy
- Best practices before adding locomotion

Feel free to open an issue or comment.
