using TestItemRunner

# TODO: Come up with a better way to test for memory leaks
# @testitem "Test for memory leaks" begin
#     function spawn_objects()
#         model, data = MuJoCo.sample_model_and_data()

#         return model.opt.timestep, data.time
#     end
#     spawn_objects() # Compile the fn
    
#     GC.gc(true)

#     before_rss = Sys.maxrss()
    
#     # Spawn a lost of objects and GC after each spawn
#     for i in 1:10
#         spawn_objects() # allocates many bytes
#         GC.gc(true)
#     end
    
#     after_rss = Sys.maxrss()

#     tolerance_bytes = 100
#     @test Int(after_rss-before_rss) <= tolerance_bytes
# end