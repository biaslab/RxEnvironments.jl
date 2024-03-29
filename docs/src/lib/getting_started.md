
# [Getting Started](@id lib-started)

When using RxEnvironments, you only have to specify the dynamics of your environment. Let's create the Bayesian Thermostat environment in `RxEnvironments`. For this example, you need `Distributions.jl` installed in your environment as well. 

Let's create the basics of our environment:

```@example thermostat
using RxEnvironments
using Distributions

# Empty agent, could contain states as well
struct ThermostatAgent end

mutable struct BayesianThermostat{T}
    temperature::T
    min_temp::T
    max_temp::T
end

# Helper functions
temperature(env::BayesianThermostat) = env.temperature
min_temp(env::BayesianThermostat) = env.min_temp
max_temp(env::BayesianThermostat) = env.max_temp
noise(env::BayesianThermostat) = Normal(0.0, 0.1)
set_temperature!(env::BayesianThermostat, temp::Real) = env.temperature = temp
function add_temperature!(env::BayesianThermostat, diff::Real) 
    env.temperature += diff
    if temperature(env) < min_temp(env)
        set_temperature!(env, min_temp(env))
    elseif temperature(env) > max_temp(env)
        set_temperature!(env, max_temp(env))
    end
end
```

By implementing `RxEnvironments.receive!`, `RxEnvironments.what_to_send` and `RxEnvironments.update!` for our environment, we can fully specify the behaviour of our environment, and `RxEnvironments` will take care of the rest. The `RxEnvironments.receive!` and `RxEnvironments.what_to_send` functions have a specific signature: `RxEnvironments.receive!(receiver, emitter, action)` takes as arguments the recipient of the action (in this example the environment), the emitter of the action (in this example the agent) and the action itself (in this example the change in temperature). The `receive!` function thus specifiec how an action from `emitter` to `recipient` affects the state of `recipient`. Always make sure to dispatch on the types of your environments, agents and actions, as `RxEnvironments` relies on Julia's multiple dispatch system to call the correct functions. Similarly for `what_to_send`, which takes the `recipient` and `emitter` (and potentially `observation`) as arguments, that computes the observation from `emitter` presented to `recipient` (when emitter has received `observation`). In our Bayesian Thermostat example, these functions look as follows:

```@example thermostat
# When the environment receives an action from the agent, we add the value of the action to the environment temperature.
RxEnvironments.receive!(recipient::BayesianThermostat, emitter::ThermostatAgent, action::Real) = add_temperature!(recipient, action)

# The environment sends a noisy observation of the temperature to the agent.
RxEnvironments.what_to_send(recipient::ThermostatAgent, emitter::BayesianThermostat) = temperature(emitter) + rand(noise(emitter))

# The environment cools down over time.
RxEnvironments.update!(env::BayesianThermostat, elapsed_time)= add_temperature!(env, -0.1 * elapsed_time)
```

Now we've fully specified our environment, and we can interact with it. In order to create the environment, we use the `RxEnvironment` struct, and we add an agent to this environment using `add!`:

```@example thermostat
environment = RxEnvironment(BayesianThermostat(0.0, -10.0, 10.0); emit_every_ms = 900)
agent = add!(environment, ThermostatAgent())
```

Now we can have the agent conduct actions in our environment. Let's have the agent conduct some actions, and inspect the observations that are being returned by the environment:

```@example thermostat
# Subscribe a logger actor to the observations of the agent
RxEnvironments.subscribe_to_observations!(agent, RxEnvironments.logger())

# Conduct 10 actions:
for i in 1:10
    action = rand()
    RxEnvironments.send!(environment, agent, action)
    sleep(1)
end
```

Congratulations! You've now implemented a basic environment in `RxEnvironments`.
