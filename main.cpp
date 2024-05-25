#include <iostream>

#include "gui/gui.h"
#include "gui/ImGuiExtensions.h"
#include "External/ImGui/imgui_internal.h"
#include "Helpers.h"
#include "structures.h"

const char* INSTITUTIONS = "Szpital\0Szkoła\0";

// Put anything other than -1 here to stop the popup from showing up
static int population = -1;

#define CITY_POINTS 50
#define CITY_SCALE 0.8f

std::vector<VoronoiDiagram> voronois(2);

int OnGui()
{
    if(population == -1) {
        population = 0;
        ImGui::OpenPopup("Ustawienia początkowe##Set Up Popup");
    }

    ImGuiStyle& style = ImGui::GetStyle();

    static int selected_type = 0;

    // Main voronoi diagram used for visualization
    static VoronoiDiagram* sample_voronoi = &voronois[selected_type];//VoronoiDiagram("../python/points.dat", "../python/vertices.dat", "../python/regions.dat");
    static TriangulationMesh mesh = TriangulationMesh("../python/points.dat", TriangulationMesh::Triangulate_Delaunay);

    static bool dirty = true; // denotes whether the parameters have changed
    static bool show_areas = false;

    static Vec2* dragging_point = nullptr;

    if(!ImGui::IsMouseDown(0))
        dragging_point = nullptr;

    // Controls
    if(ImGui::BeginChild("Parameters", {500, -1}, ImGuiChildFlags_Border)) {

        ImGui::SeparatorText("Parametry");

        //ImGui::SliderInt("Populacja", &population, 0, 10000);

        if(ImGui::Combo("Instytucja", &selected_type, INSTITUTIONS)) {
            sample_voronoi = &voronois[selected_type];
            sample_voronoi->RecalculateVoronoi();
        }

        if(ImGui::Button("Recalculate"))
            dirty = true;

        ImGui::Checkbox("Pokaż pola", &show_areas);

        ImGui::Text("Populacja: %i", population);
    }
    ImGui::EndChild();

    // Used to update data only when the parameters have changed
    if(dirty) {
        sample_voronoi->RecalculateVoronoi();
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
        ImGui::DrawVoronoi(*sample_voronoi, avail, startPos);

        if(show_areas)
            for(int i = 0; i < sample_voronoi->elements.size(); i++) {
                for (int e : sample_voronoi->elements[i]) {
                    dl->PathLineTo(ImGui::Local2Canvas(sample_voronoi->vtx[e], avail, startPos));
                }
                dl->PathFillConvex(ImColor::HSV(0/360.f, 45/100.f, sqrtf(sample_voronoi->areas[i] / (CANVAS_AREA * CITY_SCALE)), 0.3f));

                //char _buf[16];
                //sprintf(_buf, "%i", i);
                //ImGui::DrawPoint(ImGui::Local2Canvas(sample_voronoi->elements_midpoints[i], avail, startPos), _buf, dl, 4);
            }

        for(int i = 3; i < sample_voronoi->pc.points.size(); i++) {
            char _buf[16];

            auto& p = sample_voronoi->pc.points[i];
            //sprintf(_buf, "%i", i);
            if(ImGui::DrawPoint(ImGui::Local2Canvas(p, avail, startPos), "", dl, 5, POINT_SPECIAL_COLOR) && ImGui::IsMouseClicked(0)) {
                dragging_point = &p;
            }

            Vec2 pos = ImGui::Local2Canvas(sample_voronoi->pc.points[i], avail, startPos);
            Rect bb = {pos - Vec2(50, 20), pos + Vec2(50, 0)};
            //char _buf[32];
            sprintf(_buf, "%i", static_cast<int>(round(sample_voronoi->areas[i-3] / CANVAS_AREA * (float)population)));
            ImGui::RenderTextClipped(bb.min, bb.max, _buf, nullptr, nullptr, {0.5, 0.5});
        }

        // Display population on each element
        //for(int i = 0; i < sample_voronoi->elements.size(); i++) {
        //    Vec2 pos = ImGui::Local2Canvas(sample_voronoi->elements_midpoints[i], avail, startPos);
        //    Rect bb = {pos - Vec2(50, 20), pos + Vec2(50, 20)};
        //    char _buf[32];
        //    sprintf(_buf, "%i", static_cast<int>(round(sample_voronoi->areas[i] / CANVAS_AREA * (float)population)));
        //    ImGui::RenderTextClipped(bb.min, bb.max, _buf, nullptr, nullptr, {0.5, 0.5});
        //}

        //int i = 0;
        //for (const auto &item: mesh.pc.points) {
        //    char _buf[16];
        //    sprintf(_buf, "%i", i++);
        //    ImGui::DrawPoint(ImGui::Local2Canvas(item, avail, startPos), _buf, dl);
        //}

        //for(int i = 0; i < sample_voronoi->pc.points.size(); i++) {
        //    char _buf[16];
        //    sprintf(_buf, "%i", i);
        //    ImGui::DrawPoint(ImGui::Local2Canvas(sample_voronoi->pc.points[i], avail, startPos), _buf, dl);
        //}
        //
        //for(int i = 0; i < sample_voronoi->vtx.size(); i++) {
        //    char _buf[16];
        //    sprintf(_buf, "%i", i);
        //    ImGui::DrawPoint(ImGui::Local2Canvas(sample_voronoi->vtx[i], avail, startPos), _buf, dl);
        //}

        //for(int i = 0; i < mesh.elements.size(); i++) {
        //    auto& e = mesh.elements[i];
        //    Vec2 pos1 = ImGui::Local2Canvas(mesh.pc.points[e.points[0]], avail, startPos);
        //    Vec2 pos2 = ImGui::Local2Canvas(mesh.pc.points[e.points[1]], avail, startPos);
        //    Vec2 pos3 = ImGui::Local2Canvas(mesh.pc.points[e.points[2]], avail, startPos);
        //
        //    dl->AddLine(pos1, pos2, LINE_SPECIAL_COLOR, 2);
        //    dl->AddLine(pos2, pos3, LINE_SPECIAL_COLOR, 2);
        //    dl->AddLine(pos3, pos1, LINE_SPECIAL_COLOR, 2);
        //
        //    char _buf[16];
        //    sprintf(_buf, "%i", i);
        //    ImGui::DrawPoint((pos1 + pos2 + pos3) / 3, _buf, dl, 0);
        //}

        if(dragging_point) {
            *dragging_point = ImGui::Canvas2Local(ImGui::GetMousePos(), avail, startPos).Clamp(-10, 10);
            dirty = true;
        }
    }
    ImGui::EndChild();

    if(ImGui::BeginPopupModal("Ustawienia początkowe##Set Up Popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::Text("Wybierz populacje");
        //ImGui::SetNextItemWidth(-1);
        ImGui::SliderInt("##Populacja", &population, 0, 10000);

        if(ImGui::Button("Ok", {-1, 0})) {
            population = std::max(population, 0);
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    return 0;
}

/* To run with proper paths use:
cmake .
make
cd CMakeFiles/
./../VoronoiCityAnalysis
*/
int main() {
    GUI::Setup(OnGui);

    srand(time(nullptr));

    const int city_bound_size = 20;
    std::vector<PointCloud::CloudPoint> corners({{{-city_bound_size, -city_bound_size}}, {{city_bound_size, -city_bound_size}},
                                                 {{city_bound_size, city_bound_size}}, {{-city_bound_size, city_bound_size}}});

    // Populate the Voronois
    for(auto& v : voronois) {
        for(int i = 3; i < CITY_POINTS + 3; i++) {
            v.pc.points.emplace_back(Vec2(rand() * CANVAS_SIZE * 2 * CITY_SCALE / RAND_MAX,
                                       rand() * CANVAS_SIZE * 2 * CITY_SCALE / RAND_MAX)
                                       - (CANVAS_SIZE * CITY_SCALE));
        }
        // Add corner points to make "infinite" regions possible and visible
        v.pc.points.insert(v.pc.points.end(), corners.begin(), corners.end());
    }

    while (true) {
        if(GUI::RenderFrame())
            break;
    }

    GUI::ShutDown();
}
