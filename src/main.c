#define SOKOL_IMPL
#define SOKOL_GLCORE33
#include "sokol_gfx.h"
#include "sokol_gp.h"
#include "sokol_app.h"
#include "sokol_glue.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    float x, y;
    float vx, vy;
    float radius;
    sgp_color color;
} Particle;

#define MAX_PARTICLES 100
Particle particles[MAX_PARTICLES];

int showVelocityVectors = false;

// Define the gravitational constant
#define GRAVITY_CONSTANT 0.10f
#define FRICTION_CONSTANT 0.98f

static void draw_vector(float x, float y, float dx, float dy, sgp_color color)
{
    sgp_push_transform();
    sgp_set_color(color.r, color.g, color.b, color.a);
    sgp_draw_line(x, y, x + dx, y + dy);
    sgp_pop_transform();
}

static void draw_rects(void)
{
    sgp_irect viewport = sgp_query_state()->viewport;
    int width = viewport.w, height = viewport.h;
    float time = sapp_frame_count() / 60.0f;
    float t = (1.0f + sinf(time)) / 2.0f;

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        sgp_push_transform();
        // printf("[%d] %f %f %f %f\n", i, particles[i].color.r, particles[i].color.g, particles[i].color.b, particles[i].color.a);
        sgp_set_color(particles[i].color.r / 255.0f, particles[i].color.g / 255.0f, particles[i].color.b / 255.0f, 1.0f);
        sgp_draw_filled_rect(particles[i].x - particles[i].radius, particles[i].y - particles[i].radius,
                             particles[i].radius * 2.0f, particles[i].radius * 2.0f);
        sgp_pop_transform();

        if (showVelocityVectors)
        {
            // Visualize the velocity, not the attraction force
            draw_vector(particles[i].x, particles[i].y, particles[i].vx * 10, particles[i].vy * 10,
                        (sgp_color){0.0f, 0.0f, 1.0f, 1.0f}); // Blue vector for velocity
        }
    }
}

static void frame(void)
{
    // begin draw commands queue
    int width = sapp_width(), height = sapp_height();
    sgp_begin(width, height);

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        particles[i].x += particles[i].vx;
        particles[i].y += particles[i].vy;

        // Apply basic collision with screen boundaries
        if (particles[i].x < 0 || particles[i].x > width)
        {
            particles[i].vx *= -1;
        }
        if (particles[i].y < 0 || particles[i].y > height)
        {
            particles[i].vy *= -1;
        }

        particles[i].vx *= FRICTION_CONSTANT;
        particles[i].vy *= FRICTION_CONSTANT;

        for (int j = i + 1; j < MAX_PARTICLES; j++)
        {
            float dx = particles[j].x - particles[i].x;
            float dy = particles[j].y - particles[i].y;
            float distanceSquared = dx * dx + dy * dy;

            if (distanceSquared < (particles[i].radius + particles[j].radius) * (particles[i].radius + particles[j].radius))
            {
                // Collisions detected, adjust velocities
                float overlap = particles[i].radius + particles[j].radius - sqrt(distanceSquared);
                float angle = atan2(dy, dx);

                particles[i].vx -= overlap * cos(angle);
                particles[i].vy -= overlap * sin(angle);
                particles[j].vx += overlap * cos(angle);
                particles[j].vy += overlap * sin(angle);
            }
        }

        // Position and mass of the fixed point of attraction
        float attractorX = sapp_width() / 2.0f;
        float attractorY = sapp_height() / 2.0f;
        float attractorMass = 100.0f;
        float attractorRadius = 100.0f;

        // Flag to control whether gravitational attraction is enabled or not
        bool gravitationalAttractionEnabled = true;

        if (gravitationalAttractionEnabled)
        {
            for (int j = 0; j < MAX_PARTICLES; j++)
            {
                if (j == i)
                    continue; // Skip calculations for the same particle

                float dx = attractorX - particles[i].x;
                float dy = attractorY - particles[i].y;
                float distanceSquared = dx * dx + dy * dy;
                float minDistance = particles[i].radius + attractorRadius;

                if (distanceSquared < minDistance * minDistance)
                {
                    // Particle is within the combined radius, adjust position
                    float distance = sqrt(distanceSquared);
                    float penetration = minDistance - distance;
                    float angle = atan2(dy, dx);

                    // Move the particle outside the combined radius
                    particles[i].x -= penetration * cos(angle);
                    particles[i].y -= penetration * sin(angle);

                    // Apply attraction force
                    float normalizedDistance = 1.0f - (distance / attractorRadius);
                    float easedForce = GRAVITY_CONSTANT * attractorMass * normalizedDistance * normalizedDistance;
                    particles[i].vx += easedForce * cos(angle);
                    particles[i].vy += easedForce * sin(angle);
                }
                else
                {
                    // Particle is outside attractor range, apply regular attraction
                    float force = (GRAVITY_CONSTANT * attractorMass) / distanceSquared;
                    float angle = atan2(dy, dx);

                    particles[i].vx += force * cos(angle);
                    particles[i].vy += force * sin(angle);
                }
            }
        }
    }

    // draw background
    sgp_set_color(0.05f, 0.05f, 0.05f, 1.0f);
    sgp_clear();
    sgp_reset_color();

    sgp_viewport(0, 0, width, height);
    sgp_set_color(0.1f, 0.1f, 0.1f, 1.0f);
    sgp_clear();
    sgp_reset_color();
    sgp_push_transform();
    draw_rects();
    sgp_pop_transform();

    sgp_push_transform();
    sgp_set_color(1.0f, 0.0f, 0.0f, 1.0f); // Red color for the circle
    float attractorRadius = 100.0f;
    int numSides = 32;
    float angleIncrement = (2.0f * M_PI) / numSides;
    float attractorX = sapp_width() / 2.0f;
    float attractorY = sapp_height() / 2.0f;

    for (int i = 0; i < numSides; i++)
    {
        float angle = angleIncrement * i;
        float nextAngle = angle + angleIncrement;

        float x0 = attractorX + attractorRadius * cosf(angle);
        float y0 = attractorY + attractorRadius * sinf(angle);
        float x1 = attractorX + attractorRadius * cosf(nextAngle);
        float y1 = attractorY + attractorRadius * sinf(nextAngle);

        // Draw the triangle
        sgp_draw_filled_triangle(attractorX, attractorY, x0, y0, x1, y1);
    }
    sgp_reset_color();
    sgp_pop_transform();

    // dispatch draw commands
    sg_pass_action pass_action = {0};
    sg_begin_default_pass(&pass_action, width, height);
    sgp_flush();
    sgp_end();
    sg_end_pass();
    sg_commit();
}

static void init(void)
{
    // initialize Sokol GFX
    sg_desc sgdesc = {.context = sapp_sgcontext()};
    sg_setup(&sgdesc);
    if (!sg_isvalid())
    {
        fprintf(stderr, "Failed to create Sokol GFX context!\n");
        exit(-1);
    }

    // initialize Sokol GP
    sgp_desc sgpdesc = {0};
    sgp_setup(&sgpdesc);
    if (!sgp_is_valid())
    {
        fprintf(stderr, "Failed to create Sokol GP context: %s\n", sgp_get_error_message(sgp_get_last_error()));
        exit(-1);
    }

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        particles[i].x = rand() % sapp_width();
        particles[i].y = rand() % sapp_height();
        particles[i].vx = ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
        particles[i].vy = ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
        particles[i].radius = 5.0f;
        particles[i].color = (sgp_color){rand() % 256, rand() % 256, rand() % 256, 1.0};
    }
}

static void cleanup(void)
{
    sgp_shutdown();
    sg_shutdown();
}

static void event(const sapp_event *event)
{
    if (event->type == SAPP_EVENTTYPE_KEY_DOWN)
    {
        switch (event->key_code)
        {
        case SAPP_KEYCODE_S:
            showVelocityVectors = !showVelocityVectors;
            break;
        default:
            break;
        }
    }
}

sapp_desc sokol_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    return (sapp_desc){
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = event,
        .window_title = "Collision Detection",
        .sample_count = 4,
    };
}