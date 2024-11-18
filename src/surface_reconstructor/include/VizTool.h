#ifndef VISUALIZE_TOOL_H
#define VISUALIZE_TOOL_H

#include "nurbs_class.h"
#include <visualization_msgs/Marker.h>


namespace surface_reconstructor {
    class VizTool
    {
    public:
        explicit VizTool(Nurbs* surface);
        ~VizTool();

        /**
         * @brief visualize the surface.
         * @param marker The marker to be published
         * (should edit the marker's header outside of this function).
         * @param lifetime The lifetime of the marker (default is 0.0 seconds, never delete the marker).
         */
        void vizSurface(visualization_msgs::Marker &marker, double lifetime = 0.0) const;

        /**
         * @brief visualize the contact point.
         * @param u The param in u direction of the surface.
         * @param v The param in v direction of the surface.
         * @param marker The marker to be published.
         * (should edit the marker's header outside of this function).
         * @param lifetime The lifetime of the marker (default is 0.0 seconds, never delete the marker).
         */
        void vizPoint(double u, double v, visualization_msgs::Marker& marker, double lifetime = 1.0) const;

        /**
         * @brief visualize the normal.
         * @param u The param in u direction of the surface.
         * @param v The param in v direction of the surface.
         * @param marker The marker to be published.
         * (should edit the marker's header outside of this function).
         * @param lifetime The lifetime of the marker (default is 0.0 seconds, never delete the marker).
         */
        void vizNormal(double u, double v, visualization_msgs::Marker& marker, double lifetime = 1.0) const;

    private:
        Nurbs* surface_;
        bool is_viz_surface_{false};
        bool is_viz_contact_point_{false};
        bool is_viz_normal_{false};
    };
} // namespace surface_reconstructor

#endif //VISUALIZE_TOOL_H