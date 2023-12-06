#include "MainFrame.h"
#include <iostream>

namespace {

float scale = 1.f;
float aspect = 1.f;

#ifdef __APPLE__
unsigned int SCR_WIDTH = 600;
unsigned int SCR_HEIGHT = 600;
#else
unsigned int SCR_WIDTH = 1000;
unsigned int SCR_HEIGHT = 1000;
#endif

void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    scale *= std::pow(1.1f, (float)yoffset);
}

void FrameBufferSizeCallback(GLFWwindow* window, int width, int height) {
    SCR_WIDTH = width;
    SCR_HEIGHT = height;
    glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);        // Set the viewport to cover the new window                  
    aspect = (float)SCR_WIDTH / (float)SCR_HEIGHT;  // Set the aspect ratio of the clipping area to match the viewport
}

}

/**
* When user moves with the left mouse button clicked, 
* carry out a transformation on the object based on the selected mode
* Rotation / Translation / Extrusion
*/
void MainFrame::LeftMouseMove(float start_x, float start_y, float curr_x, float curr_y) {
    if (modeling_state_ == OBJ_ROTATION) {
        // ---------------------------------- Object Rotation ---------------------------------------
        glm::mat4x4 transform_mat(1.f);

        /*
            To rotate an object, we need four steps and four matrices
                1. Translation: Move the object to the origin
                2. Rotation: Rotate the object around the origin
                3. Translation: Move the object back to the point
                4. Final transformation matrix: (3) * (2) * (1)
        */

        // Step 1. Translation matrix to move the object to the origin
        // Use the centre of the object as the fixed point for translation
        glm::mat4x4 trans_from_origin_mat(1.f);
        trans_from_origin_mat[0][3] = -mesh_.center_[0];
        trans_from_origin_mat[1][3] = -mesh_.center_[1];
        trans_from_origin_mat[2][3] = -mesh_.center_[2];

        // Step 2. Rotation matrix to rotate the object around the origin
        // Previous and current mouse coordinates in screen space
        glm::vec2 s_start(start_x, start_y);
        glm::vec2 s_curr(curr_x, curr_y);

        // Compute rotation axis and normalise it
        glm::vec2 V = s_curr - s_start;
        glm::vec2 A = glm::vec2(-V.y, V.x);
        glm::vec3 rot_axis = glm::normalize(Screen2World(A + s_start) - Screen2World(s_start));

        // Compute rotation angle
        float rot_angle = 0.008f * glm::length(A);

        // Compute the rotation matrix
        glm::mat4x4 rot_mat = glm::rotate(glm::mat4x4(1.f), rot_angle, rot_axis);
        
        // Step 3. Translation matrix to move the rotation centre to the origin
        glm::mat4x4 trans_to_origin_mat(1.f);
        trans_to_origin_mat[0][3] = mesh_.center_[0];
        trans_to_origin_mat[1][3] = mesh_.center_[1];
        trans_to_origin_mat[2][3] = mesh_.center_[2];

        // Step 4. Transformation matrix involving all three steps
        // Composition of the three matrix
        transform_mat = trans_from_origin_mat * rot_mat * trans_to_origin_mat;

        // Apply transformation to the object
        mesh_.ApplyTransform(transform_mat);
    }
    else if (modeling_state_ == OBJ_TRANSLATION) {
        // ---------------------------------- Object Translation ------------------------------------
        glm::mat4x4 transform_mat(1.f);
        
        /*
            To translate an object, we need two steps and one matrix
                1. Convert the previous and current mouse positions from screen to world space
                2. Compute the translation matrix from the previous to current location
        */

        // Step 1. Compute the previous and current mouse positions in the world space
        glm::vec3 start_world = Screen2World(start_x, start_y);
        glm::vec3 curr_world = Screen2World(curr_x, curr_y);

        // Step 2. Compute the translation matrix from the previous to current position
        transform_mat = glm::translate(glm::mat4x4(1.f), curr_world - start_world);

        // Apply transformation to the object
        mesh_.ApplyTransform(transform_mat);
    }
    else if (modeling_state_ == OBJ_EXTRUDE) {
        // ---------------------------------- Face Extrusion ------------------------------------ 
        glm::mat4x4 transform_mat(1.f);
        int face_index = -1;
        
        /*
            To extrude an object, we need four steps and four matrices
                1. Compute a ray from the selected screen point
                2. Compute which face is selected on the object using ray intersection
                3. Compute the normal vector for the selected face, from the start point
                4. Compute the point on the ray to which the face should be extruded
                5. Calculate the translation matrix from the start position to the extrusion point
        */

        // Step 1. Compute a ray that goes through the mouse start position on screen
        glm::vec3 origin, R0;
        std::tie(origin, R0) = Screen2WorldRay(start_x, start_y);

        // Step 2. Compute which face on the object is selected
        glm::vec3 start_world;
        std::tie(face_index, start_world) = mesh_.FaceIntersection(origin, R0);

        // return if there is no intersection
        if (face_index == -1) return;

        // Step 3. Compute the normal vector from the intersection point
        Face& face = mesh_.faces_[face_index];
        glm::vec3 normal = glm::normalize(glm::cross(
            mesh_.vertices_[face[1]] - mesh_.vertices_[face[0]],
            mesh_.vertices_[face[2]] - mesh_.vertices_[face[1]]));
        glm::vec3 M = glm::normalize(normal);

        // Step 4. Compute the point on the ray to which the face should be extruded
        // Compute R1, the ray from the current screen point to the world
        glm::vec3 R1;
        std::tie(origin, R1) = Screen2WorldRay(curr_x, curr_y);

        // Compute V, the projection from R1 to M
        glm::vec3 V = glm::normalize(glm::cross(R1, M));
        glm::vec3 Q = glm::normalize(glm::cross(V, R1));
        float L = glm::dot(origin - start_world, Q);
        float cos = glm::dot(M, Q);
        float t = L / cos;

        // Step 4. Calculate extrusion point p_curr
        glm::vec3 curr_world = start_world + t * M;
       
        // Step 5. Calculate the translate matrix from the start position to p_curr
        transform_mat = glm::translate(glm::mat4x4(1.f), curr_world - start_world);

        // Apply transformation
        mesh_.ApplyFaceTransform(face_index, transform_mat);
    }
}

void MainFrame::VisualizeWorldSpace() {
    // ---------------------------------- World Space Visualization ------------------------------------
    // Draw xy grid 
    glLineWidth(1);
    glBegin(GL_LINES);

    glColor3f(0.2f, 0.2f, 0.2f);
    for (int x = -10; x <= 10; x++)
    {
        glVertex3f(x, -10.0f, 0.0f);
        glVertex3f(x, 10.0f, 0.0f);
    }
    for (int y = -10; y <= 10; y++)
    {
        glVertex3f(-10.0f, y, 0.0f);
        glVertex3f(10.0f, y, 0.0f);
    }
    glEnd();

    // Draw basis axes
    glLineWidth(5);
    glBegin(GL_LINES);
    
    // red line represents the x-axis
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(5.0f, 0.0f, 0.0f);

    // green line represents the y-axis
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 5.0f, 0.0f);

    // blue line represents the z-axis
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 5.0f);

    glEnd();
}


void MainFrame::MainLoop() {
    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_SAMPLES, 4);

    // glfw window creation, set viewport with width=1000 and height=1000
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "3DModeling", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
    glfwSetScrollCallback(window, ScrollCallback);
    // glad: load all OpenGL function pointers
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    const float alpha = 0.3f;
    const float beta = 0.1f;

    const float r = 5.f;
    camera_.LookAt(r * glm::vec3(std::cos(alpha) * std::cos(beta), std::cos(alpha) * std::sin(beta), std::sin(alpha)),
        glm::vec3(0.f, 0.f, 0.f),
        glm::vec3(0.f, 0.f, 1.f));

    glEnable(GL_DEPTH_TEST);

    // render loop
    while (!glfwWindowShouldClose(window)) {
        ProcessInput(window);

        // glEnable(GL_DEPTH_TEST);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Apply camera projection;
        camera_.Perspective(90.f, aspect, .5f, 10.f);
        camera_.UpdateScale(scale);
        scale = 1.f;
        camera_.ApplyProjection();

        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);        // Clear the display

        DrawScene();

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    // glfw: terminate, clearing addl previously allocated GLFW resources.
    glfwTerminate();
}

void MainFrame::ProcessInput(GLFWwindow* window) {
    // Key events
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
        modeling_state_ = OBJ_ROTATION;
    }
    if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
        modeling_state_ = OBJ_TRANSLATION;
    }
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) {
        modeling_state_ = OBJ_SUBDIVIDE;
    }
    if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS) {
        modeling_state_ = OBJ_EXTRUDE;
    }

    int current_l_mouse_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);

    // Handle left mouse
    if (current_l_mouse_state == GLFW_PRESS) {
        double xposd, yposd;
        float xpos, ypos;
        glfwGetCursorPos(window, &xposd, &yposd);
        xpos = float(xposd);
        ypos = float(SCR_HEIGHT - yposd);
        if (l_mouse_state_ == GLFW_RELEASE) {
            LeftMouseClick(xpos, ypos);
            l_click_cursor_x_ = xpos;
            l_click_cursor_y_ = ypos;
        }
        if (l_mouse_state_ == GLFW_PRESS &&
            (std::abs(xpos - last_cursor_x_) > 2.f || std::abs(ypos - last_cursor_y_) > 2.f)) {
            LeftMouseMove(l_click_cursor_x_, l_click_cursor_y_, xpos, ypos);
        }
        last_cursor_x_ = float(xpos);
        last_cursor_y_ = float(ypos);
    }
    if (current_l_mouse_state == GLFW_RELEASE) {
        if (l_mouse_state_ == GLFW_PRESS) {
            LeftMouseRelease();
        }
    }
    l_mouse_state_ = current_l_mouse_state;

    // Handle right mouse
    int current_r_mouse_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    if (current_r_mouse_state == GLFW_PRESS) {
        double xposd, yposd;
        float xpos, ypos;
        glfwGetCursorPos(window, &xposd, &yposd);
        xpos = float(xposd);
        ypos = float(SCR_HEIGHT - yposd);
        if (r_mouse_state_ == GLFW_RELEASE) {
            RightMouseClick(xpos, ypos);
        }
        if (r_mouse_state_ == GLFW_PRESS &&
            (std::abs(xpos - last_cursor_x_) > 2.f || std::abs(ypos - last_cursor_y_) > 2.f)) {
            RightMouseMove(last_cursor_x_, last_cursor_y_, xpos, ypos);
        }
        last_cursor_x_ = float(xpos);
        last_cursor_y_ = float(ypos);
    }
    if (current_r_mouse_state == GLFW_RELEASE) {
        if (r_mouse_state_ == GLFW_PRESS) {
            RightMouseRelease();
        }
    }
    r_mouse_state_ = current_r_mouse_state;
}

void MainFrame::LeftMouseClick(float x, float y) {
    if (modeling_state_ == OBJ_SUBDIVIDE) {
        glm::vec3 p_world = Screen2World(x, y);
        glm::vec3 cam_pos = camera_.view_mat_inv_ * glm::vec4(0.f, 0.f, 0.f, 1.f);
        mesh_.SubdivideFace(cam_pos, glm::normalize(p_world - cam_pos));
    }
    else if (modeling_state_ == OBJ_EXTRUDE) {
        glm::vec3 p_world = Screen2World(x, y);
        glm::vec3 cam_pos = camera_.view_mat_inv_ * glm::vec4(0.f, 0.f, 0.f, 1.f);
        mesh_.GenExtrudeFace(cam_pos, glm::normalize(p_world - cam_pos));
    }
}

void MainFrame::LeftMouseRelease() {
    mesh_.CommitTransform();
}

void MainFrame::RightMouseClick(float x, float y) {
    return;
}

void MainFrame::RightMouseMove(float start_x, float start_y, float curr_x, float curr_y) {
    glm::vec2 s_start(start_x, start_y);
    glm::vec2 s_cur(curr_x, curr_y);
    glm::vec2 V = s_cur - s_start;
    glm::vec2 A = glm::vec2(-V.y, V.x);
    glm::vec3 rot_axis = glm::normalize(Screen2World(A + s_start) - Screen2World(s_start));
    glm::mat4x4 rot_mat = glm::rotate(glm::mat4x4(1.f), 0.007f * glm::length(A), rot_axis);
    camera_.ApplyTransform(rot_mat);
}

void MainFrame::RightMouseRelease() {
    return;
}

glm::vec3 MainFrame::Camera2World(const glm::vec3& x, float w) {
    return glm::vec3(camera_.view_mat_inv_ * glm::vec4(x, w));
}

glm::vec3 MainFrame::World2Camera(const glm::vec3& x, float w) {
    return glm::vec3(camera_.view_mat_ * glm::vec4(x, w));
}

glm::vec3 MainFrame::Screen2World(const glm::vec2& v, float depth) {
    float x = v.x / SCR_WIDTH  * 2.f - 1.f;
    float y = v.y / SCR_HEIGHT * 2.f - 1.f;
    float focal = std::tan(camera_.fov_ * .5f / 180.f * glm::pi<float>());
    glm::vec4 v_camera(x * focal * aspect, y * focal, -1.f, 1.f);
    v_camera = v_camera * depth;
    glm::vec4 v_world = camera_.view_mat_inv_ * v_camera;
    return glm::vec3(v_world);
}

glm::vec3 MainFrame::Screen2World(float scr_x, float scr_y, float camera_z) {
    float x = scr_x / SCR_WIDTH * 2.f - 1.f;
    float y = scr_y / SCR_HEIGHT * 2.f - 1.f;
    float focal = std::tan(camera_.fov_ * .5f / 180.f * glm::pi<float>());
    glm::vec4 v_camera(x * focal * aspect, y * focal, -1.f, 1.f);
    v_camera = v_camera * -camera_z;
    glm::vec4 v_world = camera_.view_mat_inv_ * v_camera;
    return glm::vec3(v_world);
}

std::tuple<glm::vec3, glm::vec3> MainFrame::Screen2WorldRay(float scr_x, float scr_y) {
    float x = scr_x / SCR_WIDTH * 2.f - 1.f;
    float y = scr_y / SCR_HEIGHT * 2.f - 1.f;
    float focal = std::tan(camera_.fov_ * .5f / 180.f * glm::pi<float>());
    glm::vec3 o = camera_.view_mat_inv_ * glm::vec4(0.f, 0.f, 0.f, 1.f);
    glm::vec4 v_camera(x * focal * aspect, y * focal, -1.f, 0.f);
    glm::vec3 v = camera_.view_mat_inv_ * v_camera;
    return std::make_tuple(o, v);
}

void MainFrame::DrawScene() {
    // Draw mesh
    mesh_.Draw();

    VisualizeWorldSpace();
}