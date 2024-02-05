/**
 * @file   node_window.cpp
 * @author Dominik Authaler
 * @date   13.01.2024
 *
 * Code related to the node window within the graphical user interface.
 */

#include "node_window.hpp"

#include <imgui.h>
#include <string>

// height of the box in which the nodes are visualized
static constexpr auto BOX_HEIGHT = 500;

// utility structures and functions (definitions mainly follow at the end of the file)
struct TreeNode {
    std::string name; // node name (leaf node) / namespace (other)
    std::string fullName; // full node name for easier usage

    std::vector<std::shared_ptr<TreeNode>> children;
};

class NodeTree {
  public:
    explicit NodeTree(const std::vector<std::string> &nodes);

    std::shared_ptr<TreeNode> getRoot();

  private:
    struct SortComparator {
        inline bool operator() (const std::shared_ptr<TreeNode>& node1, const std::shared_ptr<TreeNode>& node2);
    };

    /**
     * Adds a new node within the node tree.
     * @param curNode  Root node of the (sub-)tree.
     * @param name     (Partial) node name that is considered for inserting the node within the tree.
     * @param fullName Full name of the new node (stored for convenient access to selected nodes).
     */
    void addNode(const std::shared_ptr<TreeNode>& curNode, const std::string &name, const std::string &fullName);

    /**
     * Reorders children of nodes:
     *  - leaf nodes before inner nodes
     *  - alphabetically within same groups
     */
    void sortAlphabetically(const std::shared_ptr<TreeNode>& curNode);

    std::shared_ptr<TreeNode> root;
};

void visualizeNodeTree(const std::shared_ptr<const TreeNode>& root, std::string &selectedNode);

void renderNodeWindow(const char *windowName, const std::vector<std::string> &nodeNames,
                      ServiceWrapper &serviceWrapper, std::string &selectedNode, Status &status) {
    ImGui::Begin(windowName);

    if (nodeNames.empty()) {
        ImGui::Text("No nodes available!");
    } else {
        ImGui::Text("Available nodes:");

        // organize nodes as a (sorted) tree and visualize them in a foldable structure
        NodeTree tree(nodeNames);

        // the list box creates a highlighted area in which scrolling is possible
        if (ImGui::BeginListBox("##Nodes", ImVec2(-1, BOX_HEIGHT))) {
            visualizeNodeTree(tree.getRoot(), selectedNode);
            ImGui::EndListBox();
        }
    }

    if (ImGui::Button("Refresh")) {
        serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));

        if (status.type == Status::Type::SERVICE_TIMEOUT) {
            status.text.clear();
            status.type = Status::Type::NONE;
        }
    }

    ImGui::End();
}

// implementation of utility + member functions
NodeTree::NodeTree(const std::vector<std::string> &nodes) : root(std::make_shared<TreeNode>()) {
    for (const auto &node : nodes) {
        // we ignore the leading slash for building the tree
        addNode(root, node.substr(1), node);
    }

    sortAlphabetically(root);
}

std::shared_ptr<TreeNode> NodeTree::getRoot() {
    return root;
}

void NodeTree::sortAlphabetically(const std::shared_ptr<TreeNode>& curNode) {
    for (const auto &child : curNode->children) {
        sortAlphabetically(child);
    }

    std::sort(curNode->children.begin(), curNode->children.end(), SortComparator());
}

void NodeTree::addNode(const std::shared_ptr<TreeNode> &curNode, const std::string &name, const std::string &fullName) {

    auto prefixEnd = name.find('/');
    if (prefixEnd == std::string::npos) {
        curNode->children.emplace_back(std::make_shared<TreeNode>(TreeNode{name, fullName}));
        return;
    }

    // extract first prefix and find corresponding node
    auto prefix = name.substr(0, prefixEnd);
    auto remainingName = name.substr(prefixEnd + 1);

    std::shared_ptr<TreeNode> nextNode = nullptr;
    for (const auto &child : curNode->children) {
        if (child->name.starts_with(prefix)) {
            nextNode = child;
            break;
        }
    }

    if (nextNode == nullptr) {
        nextNode = std::make_shared<TreeNode>(TreeNode{name, fullName});
        curNode->children.emplace_back(nextNode);
    } else {
        // found an existing node, check whether we have to subdivide it
        const auto idx = name.find('/');
        if (nextNode->children.empty() && idx != std::string::npos) {

            auto nextNodePrefix = nextNode->name.substr(0, idx);
            auto nextNodeRemainingName = nextNode->name.substr(idx + 1);

            nextNode->children.emplace_back(std::make_shared<TreeNode>(TreeNode{nextNodeRemainingName, nextNode->fullName}));

            nextNode->name = nextNodePrefix;
            nextNode->fullName.clear();
        }

        addNode(nextNode, remainingName, fullName);
    }
}

bool NodeTree::SortComparator::operator()(const std::shared_ptr<TreeNode> &node1,
                                          const std::shared_ptr<TreeNode> &node2) {
    bool res;

    if (node1->children.empty() && !node2->children.empty()) {
        res = true;
    } else if (!node1->children.empty() && node2->children.empty()) {
        res = false;
    } else {
        res = (node1->name < node2->name);
    }

    return res;
}

void visualizeNodeTree(const std::shared_ptr<const TreeNode>& root, std::string &selectedNode) {
    if (root->children.empty()) {
        // leaf node
        if (ImGui::Selectable(root->name.c_str())) {
            selectedNode = root->fullName;
        }
    } else {
        // inner node
        if (!root->name.empty()) {
            if (ImGui::TreeNode(root->name.c_str())) {
                for (const auto &child : root->children) {
                    visualizeNodeTree(child, selectedNode);
                }

                ImGui::TreePop();
            }
        } else {
            for (const auto &child : root->children) {
                visualizeNodeTree(child, selectedNode);
            }
        }
    }
}
