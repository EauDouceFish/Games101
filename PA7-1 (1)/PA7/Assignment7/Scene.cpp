//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }//The area is a light, then get its area. 
    //Get the pdf of the light area: summon a p to get a random Emitting obj(Light)
    // sample the area to get its pdf.
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}


//theory:Cast a ray to sample,depth is the bounce number
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    /*
    First IfIntersects
    Second Intersect with the light
    Third Intersect with the obj
    */
    Vector3f dir_illumination = {0.0, 0.0, 0.0};
    Vector3f indir_illumination = {0.0, 0.0, 0.0};
    Intersection thisIntersection = Scene::intersect(ray);

    if(!thisIntersection.happened)
    {
        return dir_illumination;
    }//First, Judge. Whether the ray intersect with an obj? light obj or simple obj?
    //If no the Ray have no intersection with the obj in scene,return 0, 0, 0


    if(thisIntersection.m->hasEmission())//Second, have intersection then Judge -->> is the light?
    {
        if(depth==0)//if depth != 0, then the ray is the bounced, no discussion here
        //The emission of the ray gets from Light(depth>0) should be discussed in the indir_illumination.
        {
            return thisIntersection.m->getEmission();
        }
        else
        {
            return dir_illumination;
        }
    }

    //Third with obj

    //First we need direct illumination, we need wi, wo, L_i, f_r, costheta(Prime), Distance, light_pdf.


    Intersection light_position;
    auto light_pdf = 0.0f;//initializing the parametres.
    sampleLight(light_position, light_pdf);
    //Sample all the lights in the world, to simplify the process.
    //Then assign the result to the light_position and light_pdf.

    Vector3f ShadePoint = thisIntersection.coords;//ShadePoint is on the obj.
    Vector3f LightPosition = light_position.coords;//get the position of sampled lights
    Vector3f ShadePointNormal = thisIntersection.normal.normalized();
    Vector3f LightPositionNormal = light_position.normal.normalized();
    Vector3f wo = -ray.direction;
    //From obj to Scene(Screen),(so vector is ScenePos - ShadePoint = -ray.direction) wo is reverse
    Vector3f wi = (ShadePoint - LightPosition).normalized();

    auto LightShadePointDistance = (ShadePoint - LightPosition).norm();
    auto LightShadePointDistance2 = dotProduct((ShadePoint - LightPosition), (ShadePoint-LightPosition));
    


    Ray LightShadePoint(LightPosition, wi);
    //Set an origin then a direction to create a Ray
    Intersection LightScene = Scene::intersect(LightShadePoint);



    //If light could cast onto the scene and the dis > LightShadePointDistance, not blocked
    if(LightScene.happened && (LightScene.distance - LightShadePointDistance > -EPSILON))
    {
        Vector3f L_i = light_position.emit;//'irradiance'

        Vector3f f_r = thisIntersection.m->eval(-wo, -wi ,ShadePointNormal);
        //This is BRDF, radiance/irradiance in dimension


        //In this frame, eval is the reverse
        //So when calculating we need to change the direction of wi, wo, so is -wo, -wi
        float costheta = dotProduct(-wi, ShadePointNormal);
        float costhetaPrime = dotProduct(wi, LightPositionNormal);
        dir_illumination = L_i*f_r*costheta*costhetaPrime/LightShadePointDistance2/light_pdf;
    }


    //Then we use RR to calculate indir_illumination
    float RRTestNumber = get_random_float();
    if(RRTestNumber < RussianRoulette)
    {
        Vector3f rand_wi = thisIntersection.m->sample(-wo, ShadePointNormal).normalized();
        //return a random direction in the hemisphere, constructing a random ray.

        Ray RandRay(ShadePoint, rand_wi);
        Intersection ShadePointToScene = Scene::intersect(RandRay);
        if(ShadePointToScene.happened && !ShadePointToScene.m->hasEmission())
        {
            Vector3f f_r = thisIntersection.m->eval(-wo, rand_wi ,ShadePointNormal);
            float cos_theta = dotProduct(rand_wi, ShadePointNormal);
            float hemi_pdf = thisIntersection.m->pdf(-wo, rand_wi, ShadePointNormal);

            indir_illumination = castRay(RandRay, depth+1)*f_r*cos_theta/hemi_pdf/RussianRoulette;

        }
    }


    return dir_illumination+indir_illumination;
    //We should shade(ShadePoint, wo)
  
   /*
        shade (p, wo)
        sampleLight ( inter , pdf_light )
        Get x, ws , NN , emit from inter
        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws ,NN) / |x-p |^2 / pdf_light

        L_indir = 0.0
        Test Russian Roulette with probability RussianRoulette
        wi = sample (wo , N)
        Trace a ray r(p, wi)
        If ray r hit a non - emitting object at q
            L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N) / pdf (wo , wi , N) / RussianRoulette

        Return L_dir + L_indir
    */
// Implementation of Path Tracing
}