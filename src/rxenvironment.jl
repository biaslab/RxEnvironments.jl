using Rocket

export RxEnvironment, add!

function RxEnvironment(
    environment;
    is_discrete::Bool=false,
    emit_every_ms::Int=1000,
    real_time_factor::Real=1.0,
)
    state_space = is_discrete ? DiscreteEntity() : ContinuousEntity()
    entity = create_entity(environment, state_space, ActiveEntity(), real_time_factor)
    if !is_discrete
        add_timer!(entity, emit_every_ms; real_time_factor=real_time_factor)
    end
    return entity
end
