function GeodesicPSIM = feature_pooling(results)

result = results;
       s=1;     
        for k = 1:1:length(result) % 
            proposed = result{k,1};  %                
                if ~isempty(proposed)
                    feature = proposed;
                    %% number of effective pixels in 1-hop geodesic patch
                    ref_n = feature.ref_vertex_eff;
                    dis_n = feature.dis_vertex_eff;
                    weighted_ref_n = ref_n./sum(ref_n);
                    weighted_dis_n = dis_n./sum(dis_n);
                    %% similarity of patch color smoothness
                    ref_smoothness = feature.ref_smoothness;
                    dis_smoothness = feature.dis_smoothness;


                    smoothness_diff4 = similarity((ref_smoothness./sum(ref_n)),(dis_smoothness./sum(dis_n)));                    
                    average_pcs(s,1) = (6*smoothness_diff4(1,1)+smoothness_diff4(1,2)+smoothness_diff4(1,3))/8;
                 
                    %% similarity of patch discrete mean curvature
                    average_dmc(s,1) = similarity((feature.ref_cur_value), (feature.dis_cur_value));
                    

                    %% similarity of patch pixel color average and variance


                    average_face_lm(s,1) = similarity(sum(weighted_ref_n.* feature.ref_facelm), sum(weighted_dis_n.* feature.dis_facelm));
                    average_face_var(s,1) = similarity(sum(weighted_ref_n.* feature.ref_facevar), sum(weighted_dis_n.* feature.dis_facevar));

                    average_face_c1m(s,1) = similarity(sum(weighted_ref_n.*feature.ref_facec1m), sum(weighted_dis_n.*feature.dis_facec1m));
                    average_face_c1var(s,1) = similarity(sum(weighted_ref_n.*feature.ref_facec1var), sum(weighted_dis_n.*feature.dis_facec1var));

                    average_face_c2m(s,1) = similarity(sum(weighted_ref_n.*feature.ref_facec2m), sum(weighted_dis_n.*feature.dis_facec2m));
                    average_face_c2var(s,1) = similarity(sum(weighted_ref_n.*feature.ref_facec2var), sum(weighted_dis_n.*feature.dis_facec2var));


                    s=s+1;
                end
        end
                
            if s>1


            score_pcs = mean(average_pcs);            
            score_dmc = nanmean(average_dmc);
            score_face_lm = mean(average_face_lm);
            score_face_var = mean(average_face_var);
            score_face_c1m = mean(average_face_c1m);
            score_face_c1var = mean(average_face_c1var);
            score_face_c2m = mean(average_face_c2m);
            score_face_c2var = mean(average_face_c2var);
                
            end
                  
score.dmc = score_dmc;
score.pcs = score_pcs;
score.face_lm = score_face_lm;
score.face_var = score_face_var;
score.face_c1m =score_face_c1m;
score.face_c1var=score_face_c1var;
score.face_c2m = score_face_c2m;
score.face_c2var = score_face_c2var;
obj_score = score;


dmc = real(obj_score.dmc);
pcs = real(obj_score.pcs);
face_lm = real(obj_score.face_lm);
face_var = real(obj_score.face_var);
face_c1m = real(obj_score.face_c1m);
face_c1var = real(obj_score.face_c1var);
face_c2m = real(obj_score.face_c2m);
face_c2var = real(obj_score.face_c2var);
pcv = (6*face_var + face_c1var +face_c2var)/8;
pca = (6*face_lm + face_c1m +face_c2m)/8;


GeodesicPSIM = (pca + dmc + pcs + pcv)/4 ;