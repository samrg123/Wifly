#pragma once

#include <vector>
#include <initializer_list>
#include <algorithm>

template<typename Callback>
struct ChainedCallback;

template<typename... ArgsT>
struct ChainedCallback<void(*)(ArgsT...)> {

    using Callback = void(*)(ArgsT...);
    std::vector<Callback> callbacks;

    ChainedCallback() {}

    template<size_t kN>
    ChainedCallback(const Callback (&callbacks_)[kN]): callbacks(callbacks_, callbacks_+kN) {}  

    inline void Append(Callback callback) {
        callbacks.push_back(callback);
    }

    inline auto Find(Callback callback) {
        return std::find(callbacks.begin(), callbacks.end(), callback);
    }

    inline auto Find(Callback callback) const {
        return std::find(callbacks.begin(), callbacks.end(), callback);
    }

    inline bool Contains(Callback callback) const {
        return Find(callback) != callbacks.end();
    }

    inline bool Remove(Callback callback) {

        auto it = Find(callback);
        if(it == callbacks.end()) return false;

        callbacks.erase(it);
        return true;
    }

    template<typename IteratorT>
    inline void Remove(IteratorT iterator) {
        callbacks.erase(iterator);
    }    

    //Replaces the oldcallback with the specified new callback. If the oldcallback isn't in the 
    //callbacks array the newCallback is appended to the back and the function returns false. 
    //returns true if oldCallback was replaced, false otherwise 
    inline bool Replace(Callback oldCallback, Callback newCallback) {
        for(Callback& callback : callbacks) {
            if(callback == oldCallback) {
                callback = newCallback;
                return true;
            }
        }

        callbacks.push_back(newCallback);
        return false;
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
