#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; i++)
        {
            Vector2D cur = start + i * ((end - start) / (num_nodes - 1));
            masses.push_back(new Mass(cur, node_mass, false));
        }
        for (int i = 0; i < masses.size() - 1; i++)
        {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }
        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) 
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto dist = s->m2->position - s->m1->position;
            auto re_pos = dist.norm() - s->rest_length;
            auto re_v = dot(s->m1->velocity - s->m2->velocity, dist / dist.norm()) * (dist / dist.norm());
            auto re_v_t = re_v - (s->m1->velocity - s->m2->velocity);

            if (re_pos > 0.2f * s->rest_length) 
                re_pos = s->rest_length;
            if (!s->m1->pinned){
                s->m1->forces += s->k * dist / dist.norm() * re_pos - re_v * 0.05 + re_v_t * 0.05;
            }
                
            if (!s->m2->pinned){
                s->m2->forces += -s->k * dist / dist.norm() * re_pos + re_v * 0.05 - re_v_t * 0.05;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                float damping_factor = 0.00005;
                m->forces += -damping_factor * m->velocity;
                m->velocity += delta_t * (m->forces / m->mass);
                m->position += delta_t * m->velocity;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto dist = s->m2->position - s->m1->position;
            auto re_pos = dist.norm() - s->rest_length;
            auto re_v = dot(s->m1->velocity - s->m2->velocity, dist / dist.norm()) * (dist / dist.norm());
            auto re_v_t = re_v - (s->m1->velocity - s->m2->velocity);

            if (re_pos > 0.2f * s->rest_length) 
                re_pos = s->rest_length;
            if (!s->m1->pinned){
                s->m1->forces += s->k * dist / dist.norm() * re_pos - re_v * 0.05 + re_v_t * 0.05;
            }
                
            if (!s->m2->pinned){
                s->m2->forces += -s->k * dist / dist.norm() * re_pos + re_v * 0.05 - re_v_t * 0.05;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005;
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position) + (m->forces / m->mass) * delta_t * delta_t;

                // m->velocity = delta_t * (m->forces / m->mass);
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
