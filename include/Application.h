//
// Created by brice on 10/4/25.
//

#ifndef VISUAL_APPLICATION_H
#define VISUAL_APPLICATION_H

#include "Engine.h"

/*
 * Creates a window and an engine that handle render logic
 */
class Application {
public:

    // ------------------------ Constructors and Destructors ------------------------ //

    Application();

    // -------------------------------- Main methods -------------------------------- //

    void Run();

    // ---------------------------- Getters and Setters ---------------------------- //

    Engine &GetEngine();

private:

    void InitImGUI() const;

    // ---------------------------------- Statics ---------------------------------- //

    static GLFWwindow* CreateWindow();

    // ---------------------------------- Members ---------------------------------- //

    std::unique_ptr<Engine> _engine;
    GLFWwindow *_window = nullptr;

};


#endif //VISUAL_APPLICATION_H