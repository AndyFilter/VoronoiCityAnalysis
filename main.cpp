#include <iostream>

#include "gui/gui.h"
#include "gui/ImGuiExtensions.h"
#include "External/ImGui/imgui_internal.h"
#include "Helpers.h"
#include "structures.h"

const char* INSTITUTIONS = "Szpital\0Szkoła\0";

int OnGui()
{
    ImGuiStyle& style = ImGui::GetStyle();

    // Main voronoi diagram used for visualization
    static VoronoiDiagram sample_voronoi = VoronoiDiagram("../python/points.dat", "../python/vertices.dat", "../python/regions.dat");

    static bool dirty = false; // denotes whether the parameters have changed
    static int population;
    static int selected_type = 0;

    static Vec2* dragging_point = nullptr;

    if(!ImGui::IsMouseDown(0))
        dragging_point = nullptr;

    // Controls
    if(ImGui::BeginChild("Parameters", {500, -1}, ImGuiChildFlags_Border)) {

        ImGui::SeparatorText("Parametry");

        ImGui::SliderInt("Populacja", &population, 0, 10000);

        dirty |= ImGui::Combo("Instytucja", &selected_type, INSTITUTIONS);
    }
    ImGui::EndChild();

    // Used to update data only when the parameters have changed
    if(dirty) {
        sample_voronoi.RecalculateVoronoi();
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
        ImGui::DrawVoronoi(sample_voronoi, avail, startPos);

        for(auto& p : sample_voronoi.points) {
            if(ImGui::DrawPoint(ImGui::Local2Canvas(p, avail, startPos), "", dl, 5, POINT_SPECIAL_COLOR) && ImGui::IsMouseClicked(0)) {
                dragging_point = &p;
            }
        }

        if(dragging_point) {
            *dragging_point = ImGui::Canvas2Local(ImGui::GetMousePos(), avail, startPos).Clamp(-10, 10);
            dirty = true;
        }
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
