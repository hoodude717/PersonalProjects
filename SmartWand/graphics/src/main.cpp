#include "config.h"
#include "triangle_mesh.h"
#include "material.h"
#include "lin_algebra.h"



unsigned int make_module(const std::string& filepath, unsigned int module_type);

unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath);

const std::string vertex_filepath = "../graphics/src/shaders/vertex.txt";
const std::string fragment_filepath = "../graphics/src/shaders/fragment.txt";

int main() {


    GLFWwindow* window;
    if (!glfwInit()) {
        std::cout << "GLFW Couldnt Initialize" << std::endl;
        return -1;
    }
    window = glfwCreateWindow(640,480,"Smart Wand Display", NULL, NULL);
    if (!window) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate();
        return -1;
    }

    glClearColor(0.0f, 0.0f, 0.0f, .50f);
    unsigned int shaderProgram = make_shader(vertex_filepath, fragment_filepath);

    TriangleMesh* triangle = new TriangleMesh();
    Material* material = new Material("../graphics/img/Hogwarts.jpeg");
    Material* mask = new Material("../graphics/img/border.jpg");
    
    glUseProgram(shaderProgram);
    glUniform1i(glGetUniformLocation(shaderProgram, "material"), 0);
    glUniform1i(glGetUniformLocation(shaderProgram, "mask"), 1);


    vec3 quad_pos = {0.4f, -0.2f, 0.0f};
    // mat4 model = create_matrix_transform(quad_pos);
    unsigned int model_loc = glGetUniformLocation(shaderProgram, "model");
    // glUniformMatrix4fv(model_loc, 1, GL_FALSE, model.entries);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        mat4 model = create_z_rot(10* glfwGetTime());
        glUniformMatrix4fv(model_loc, 1, GL_FALSE, model.entries);


        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        material->use(0);
        mask->use(1);
        triangle->draw();
        glfwSwapBuffers(window);

    }
    glDeleteProgram(shaderProgram);
    delete triangle;
    delete material;
    delete mask;
    glfwTerminate();
    return 0;
}

unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath) {
    std::vector<unsigned int> shaderModules;
    shaderModules.push_back(make_module(vertex_filepath, GL_VERTEX_SHADER));
    shaderModules.push_back(make_module(fragment_filepath, GL_FRAGMENT_SHADER));

    unsigned int shaderProgram = glCreateProgram();
    for (unsigned int module : shaderModules) {
        glAttachShader(shaderProgram, module);
    }
    glLinkProgram(shaderProgram);

    int success; 
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetProgramInfoLog(shaderProgram, 1024, NULL, errorLog);
        std::cout << "Error linking shader program: " << std::endl << errorLog ;
    }

    for (unsigned int module : shaderModules) {
        glDeleteShader(module);
    }
    return shaderProgram;
}

unsigned int make_module(const std::string& filepath, unsigned int module_type) {
    std::ifstream file(filepath);
    std::stringstream bufferLines;
    bufferLines << file.rdbuf(); // Read the entire file in one call


    std::string shaderSource = bufferLines.str();
    const char* shaderCode = shaderSource.c_str();
    
    file.close();

    unsigned int shaderModule = glCreateShader(module_type);
    if (shaderModule == 0) {
        std::cout << "Failed to create shader object for type: " << module_type << std::endl;
        return 0;
    }
    glShaderSource(shaderModule, 1, &shaderCode, NULL);
    glCompileShader(shaderModule);

    int success; 
    glGetShaderiv(shaderModule, GL_COMPILE_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetShaderInfoLog(shaderModule, 1024, NULL, errorLog);
        std::cout << "Error compiling shader module: " << errorLog << std::endl;
    }
    bufferLines.clear();
    return shaderModule;
}
