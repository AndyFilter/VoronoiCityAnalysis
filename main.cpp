#include <iostream>

#include "../gui/gui.h"
#include "../gui/ImGuiExtensions.h"
#include "../External/ImGui/imgui_internal.h"
#include "../Helpers.h"

int OnGui()
{
    ImGuiStyle& style = ImGui::GetStyle();
    static bool dirty = false; // denotes whether the parameters have changed

    // Controls
    if(ImGui::BeginChild("Parameters", {500, -1}, ImGuiChildFlags_Border)) {

        ImGui::SeparatorText("Parametry");
    }
    ImGui::EndChild();

    // Used to update data only when the parameters have changed
    if(dirty) {
        dirty = false;
    }

    ImGui::SameLine();

    // Canvas coordinates system [-10, 10]
    if(ImGui::BeginChild("Canvas", {-1, -1}, ImGuiChildFlags_Border)) {
        auto dl = ImGui::GetWindowDrawList();
        Vec2 avail = (Vec2)ImGui::GetContentRegionAvail() + (Vec2)style.WindowPadding * 2; // Canvas available space
        auto startPos = (Vec2)ImGui::GetWindowPos(); // Canvas window start pos (local)

        ImGui::DrawCanvas(dl, startPos, avail);

        avail += Vec2(-20, -20);
        startPos += Vec2(10, 10);
        /* These two variables are used as a transform data to go from local to canvas or vice versa */
        /* For example ImGui::DrawPoint(ImGui::Local2Canvas(point, avail, startPos)); */

        // Actual code here



    }
    ImGui::EndChild();


    return 0;
}

int main() {
    GUI::Setup(OnGui);

    srand(time(nullptr));

    while (true) {
        if(GUI::RenderFrame())
            break;
    }

    GUI::ShutDown();
}
