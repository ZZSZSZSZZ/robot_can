/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-02
 * @Description: 
 * @Version: 1.0
 */

#include <string>
#include <vector>

#include "motor/motor_type.hpp"

namespace robot {
    class BaseComponent {
    public:
        virtual ~BaseComponent() = default;

        virtual void initialize(const std::vector<motor::MotorConfig> &motor_configs) = 0;

        virtual void shutdown() = 0;

        virtual bool enableAll() const = 0;

        virtual bool disableAll() const = 0;

        virtual std::string getName() const = 0;
    };
}
