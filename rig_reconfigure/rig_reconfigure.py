import imviz as viz
import sys
import subprocess

class ReconfigureGUI:
    def __init__(self):
        self.available_nodes = []
        self.selected_node = 0
        self.node_parameters = None
        self.previously_selected_node = None

    def __autogui__(self, **kwargs):
        if not viz.wait():
            sys.exit()

        viz.set_main_window_title("ROS2 ImGui Reconfiguration")
        if viz.begin_window("Parameter Reconfiguration"):
            viz.text("Node selection:     ")
            viz.same_line()
            self.selected_node = viz.combo("##", self.available_nodes, self.selected_node)

            viz.same_line()

            if viz.button("Refresh"):
                self.available_nodes = self.query_nodes()

                # update the parameter list as well by resetting the previously visualized node
                self.previously_selected_node = None

            if len(self.available_nodes) > 0:
                selected_node_name = self.available_nodes[self.selected_node]

                if self.previously_selected_node != selected_node_name:
                    self.previously_selected_node = selected_node_name
                    self.node_parameters = self.query_node_parameters(selected_node_name)

            if self.node_parameters is not None:
                viz.separator()
                viz.text("Parameters\t\t")
#                viz.same_line()
#                if viz.button("Dump to file"):
#                    self.save_parameters()

            viz.end_window()

    def query_nodes(self):
        # TODO: maybe replace later on by the corresponding service call (once found)
        result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE)

        result_string = result.stdout.decode("utf-8")

        nodes = result_string.split('\n')

        return nodes[:-1] # last one is empty newline

    def query_node_parameters(self, node_name):
        print(f"Querying parameters for node '{node_name}'")
        return []

    def save_parameters(self):
        print("Saving parameters")


def main(args=None):
    viz.dev.launch(ReconfigureGUI, "__autogui__")

if __name__ == '__main__':
    main()
