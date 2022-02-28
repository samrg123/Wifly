#pragma once

#include <list>

template<typename Callback>
struct ChainedCallback;

template<typename... ArgsT>
struct ChainedCallback<void(*)(ArgsT...)> {

    using Callback = void(*)(ArgsT...);
    std::list<Callback> callbacks;

    inline void Append(Callback callback) {
        callbacks.push_back(callback);
    }

    inline void Remove(Callback callback) {
        callbacks.remove(callback);
    }

    inline void Clear() {
        callbacks.clear();
    }


    inline void operator()(ArgsT... args) {
    
        for(Callback callback : callbacks) {
            
            //skip null callbacks
            if(UNLIKELY(!callback)) continue;

            callback(args...);
        }
    }
};
