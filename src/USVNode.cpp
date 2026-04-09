#include "USVNode.hpp"

    USVNode::USVNode() : Node("usv_controller")
    {}
    
    void USVNode::init(){
        this->usv_ = std::make_unique<USV>(this->shared_from_this());
    }
