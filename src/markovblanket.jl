using Rocket
using Dictionaries
import Dictionaries: Dictionary

struct Actuator
    subject::Rocket.RecentSubjectInstance
end

Actuator() = Actuator(RecentSubject(Any))

emission_channel(actuator::Actuator) = actuator.subject
send_action!(actuator::Actuator, action) = next!(emission_channel(actuator), action)

Rocket.subscribe!(actuator::Actuator, actor::Rocket.Actor{T} where {T}) =
    subscribe!(emission_channel(actuator), actor)

struct SensorActor <: Rocket.Actor{Any}
    emitter::AbstractEntity
    receiver::AbstractEntity
end

emitter(actor::SensorActor) = actor.emitter
receiver(actor::SensorActor) = actor.receiver

Rocket.on_next!(actor::SensorActor, stimulus) =
    receive_observation!(receiver(actor), Observation(emitter(actor), stimulus))
Rocket.on_error!(actor::SensorActor, error) = println("Error in SensorActor: $error")
Rocket.on_complete!(actor::SensorActor) = println("SensorActor completed")

struct Sensor
    actor::SensorActor
    subscription::Teardown
end

Sensor(entity::AbstractEntity, emitter::AbstractEntity) =
    Sensor(SensorActor(entity, emitter))
Sensor(actor::SensorActor) =
    Sensor(actor, subscribe!(get_actuator(emitter(actor), receiver(actor)), actor))
Rocket.unsubscribe!(sensor::Sensor) = Rocket.unsubscribe!(sensor.subscription)

struct Observations{T}
    state_space::T
    buffer::AbstractDictionary{Any, Union{Observation, Nothing}}
    target::Rocket.RecentSubjectInstance
end

Observations(state_space::Discrete) = Observations(state_space, Dictionary{Any, Union{Observation, Nothing}}(), RecentSubject(ObservationCollection))
Observations(state_space::Continuous) = Observations(state_space, Dictionary{Any, Union{Observation, Nothing}}(), RecentSubject(AbstractObservation))
target(observations::Observations) = observations.target
function clear_buffer!(observations::Observations) 
    for (key, value) in pairs(observations.buffer)
        observations.buffer[key] = nothing
    end
end

Rocket.subscribe!(observations::Observations, actor::Rocket.Actor{T} where T) = subscribe!(target(observations), actor)

Rocket.next!(observations::Observations{Continuous}, observation::Union{Observation, TimerMessage}) = next!(target(observations), observation)
function Rocket.next!(observations::Observations{Discrete}, observation::Observation)
    observations.buffer[emitter(observation)] = observation
    if sum(values(observations.buffer) .== nothing) == 0
        next!(target(observations), ObservationCollection(Tuple(observations.buffer)))
        clear_buffer!(observations)
    end
end
function Rocket.next!(observations::Observations{Discrete}, observation::TimerMessage)
    @error "Clocked environment not supported for Discrete state space"
end

struct MarkovBlanket{T}
    actuators::AbstractDictionary{Any,Actuator}
    sensors::AbstractDictionary{Any,Sensor}
    observations::Observations{T}
end

MarkovBlanket(state_space) = MarkovBlanket(
    Dictionary{Any,Actuator}(),
    Dictionary{Any,Sensor}(),
    Observations(state_space),
)

actuators(markov_blanket::MarkovBlanket) = markov_blanket.actuators
sensors(markov_blanket::MarkovBlanket) = markov_blanket.sensors
observations(markov_blanket::MarkovBlanket) = markov_blanket.observations

function get_actuator(markov_blanket::MarkovBlanket, agent::AbstractEntity)
    if !haskey(actuators(markov_blanket), agent)
        throw(NotSubscribedException(markov_blanket, agent))
    end
    return actuators(markov_blanket)[agent]
end

add_to_state!(entity, to_add) = nothing

function add_sensor!(markov_blanket::MarkovBlanket{Discrete}, emitter::AbstractEntity, receiver::AbstractEntity)
    sensor = Sensor(emitter, receiver)
    insert!(sensors(markov_blanket), emitter, sensor)
    insert!(observations(markov_blanket).buffer, emitter, nothing)
end

function add_sensor!(markov_blanket::MarkovBlanket{Continuous}, emitter::AbstractEntity, receiver::AbstractEntity)
    sensor = Sensor(emitter, receiver)
    insert!(sensors(markov_blanket), emitter, sensor)
end

function Rocket.subscribe!(emitter::AbstractEntity, receiver::AbstractEntity)
    actuator = Actuator()
    insert!(actuators(markov_blanket(emitter)), receiver, actuator)
    add_sensor!(markov_blanket(receiver), emitter, receiver)
    add_to_state!(entity(emitter), entity(receiver))
end

function Rocket.subscribe!(emitter::AbstractEntity, receiver::Rocket.Actor{T} where {T})
    actuator = Actuator()
    insert!(actuators(emitter), receiver, actuator)
    return subscribe!(actuator, receiver)
end

function Rocket.unsubscribe!(emitter::AbstractEntity, receiver::AbstractEntity)
    Rocket.unsubscribe!(sensors(receiver)[emitter])
    delete!(sensors(receiver), emitter)
    delete!(actuators(emitter), receiver)
end

function Rocket.unsubscribe!(
    emitter::AbstractEntity,
    actor::Rocket.Actor,
    subscription::Teardown,
)
    delete!(actuators(emitter), actor)
    unsubscribe!(subscription)
end

function conduct_action!(emitter::AbstractEntity, receiver::AbstractEntity, action::Any)
    actuator = get_actuator(emitter, receiver)
    send_action!(actuator, action)
end


function receive_observation!(entity::AbstractEntity, observation::Observation)
    next!(observations(entity), observation)
end